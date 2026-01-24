// mjc_fragment.h
// UDP packet fragmentation and reassembly for FlatBuffers messages
// Handles messages larger than 1500 MTU by splitting into fragments

#ifndef mjc_fragment_h
#define mjc_fragment_h

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// MARK: - Constants

// Network MTU and payload calculations
// NOTE: These values assume IPv4 with no IP options (20-byte header) and 8-byte UDP header.
// For IPv6 (40-byte header) or IPv4 with options, constants would need adjustment.
// All multi-byte header fields use native byte order (no network byte order conversion).
// This protocol is intended for same-architecture local network communication.
#define MJ_MTU                  1500
#define MJ_IP_UDP_OVERHEAD      28      // 20 bytes IP + 8 bytes UDP
#define MJ_MAX_UDP_PAYLOAD      1472    // MTU - IP/UDP overhead (space for header + payload)
#define MJ_FRAGMENT_HEADER_SIZE 16
#define MJ_MAX_FRAGMENT_PAYLOAD 1456    // Max UDP payload - fragment header
#define MJ_MAX_FRAGMENTS        255
#define MJ_MAX_MESSAGE_SIZE     (MJ_MAX_FRAGMENTS * MJ_MAX_FRAGMENT_PAYLOAD)  // ~371KB

// Fragment packet magic number: "MJFG" (MuJoCo Fragment)
#define MJ_PACKET_MAGIC_FRAG    0x4D4A4647

// Reassembly configuration
#define MJ_REASSEMBLY_SLOTS     8       // Max concurrent incomplete messages
#define MJ_REASSEMBLY_TIMEOUT_MS 100    // Timeout for incomplete messages

// MARK: - Fragment Header

// Fragment header structure (16 bytes, packed)
// Placed at the start of each UDP packet containing a fragment
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;           // MJ_PACKET_MAGIC_FRAG (0x4D4A4647)
    uint16_t message_id;      // Unique ID correlating fragments of same message
    uint8_t  fragment_index;  // 0-based position in sequence (0-254)
    uint8_t  fragment_count;  // Total number of fragments (1-255)
    uint32_t total_size;      // Total size of complete message payload
    uint16_t payload_size;    // Size of payload in this fragment
    uint16_t checksum;        // CRC-16 of header (excluding checksum field)
} MJFragmentHeader;
#pragma pack(pop)

// MARK: - CRC-16 Utility

/// Compute CRC-16 checksum for fragment header validation
/// Uses CRC-16-CCITT polynomial (0x1021)
uint16_t mj_fragment_crc16(const uint8_t* data, size_t length);

/// Compute checksum for a fragment header (excludes checksum field itself)
uint16_t mj_fragment_header_checksum(const MJFragmentHeader* header);

/// Validate a fragment header's checksum
bool mj_fragment_header_valid(const MJFragmentHeader* header);

#ifdef __cplusplus
}
#endif

// MARK: - C++ Classes

#ifdef __cplusplus

#include <atomic>
#include <vector>
#include <array>
#include <chrono>
#include <netinet/in.h>

namespace imujoco {

// MARK: - FragmentedSender

/// Splits large messages into MTU-sized fragments and sends via UDP.
/// @note NOT thread-safe. Each thread should use its own FragmentedSender instance,
///       or external synchronization must be provided.
class FragmentedSender {
public:
    FragmentedSender() = default;
    ~FragmentedSender() = default;

    // Non-copyable
    FragmentedSender(const FragmentedSender&) = delete;
    FragmentedSender& operator=(const FragmentedSender&) = delete;

    /// Fragment and send a message over UDP
    /// @param data Pointer to message data (typically FlatBuffers buffer)
    /// @param size Size of message in bytes
    /// @param socket_fd UDP socket file descriptor
    /// @param dest Destination address
    /// @return Number of fragments sent, or -1 on error
    int SendMessage(const uint8_t* data, size_t size,
                    int socket_fd, const struct sockaddr_in& dest);

    /// Get the next message ID that will be used
    uint16_t GetNextMessageId() const { return message_id_.load(std::memory_order_relaxed); }

private:
    std::atomic<uint16_t> message_id_{0};
    std::vector<uint8_t> fragment_buffer_;
};

// MARK: - ReassemblySlot

/// Internal structure for tracking incomplete message reassembly
struct ReassemblySlot {
    uint16_t message_id = 0;
    uint8_t fragment_count = 0;
    uint8_t received_count = 0;
    uint32_t total_size = 0;

    // Bitmask for received fragments (supports up to 64 fragments directly)
    uint64_t received_mask = 0;
    // Extended bitmask for fragments 64-255 (192 more bits = 24 bytes)
    uint8_t extended_mask[24] = {0};

    std::vector<uint8_t> buffer;
    std::chrono::steady_clock::time_point last_activity;
    bool active = false;

    void Reset() {
        message_id = 0;
        fragment_count = 0;
        received_count = 0;
        total_size = 0;
        received_mask = 0;
        memset(extended_mask, 0, sizeof(extended_mask));
        buffer.clear();
        active = false;
    }

    bool IsFragmentReceived(uint8_t index) const {
        if (index < 64) {
            return (received_mask & (1ULL << index)) != 0;
        } else {
            uint8_t ext_index = index - 64;
            uint8_t byte_index = ext_index / 8;
            uint8_t bit_index = ext_index % 8;
            return (extended_mask[byte_index] & (1 << bit_index)) != 0;
        }
    }

    void MarkFragmentReceived(uint8_t index) {
        if (index < 64) {
            received_mask |= (1ULL << index);
        } else {
            uint8_t ext_index = index - 64;
            uint8_t byte_index = ext_index / 8;
            uint8_t bit_index = ext_index % 8;
            extended_mask[byte_index] |= (1 << bit_index);
        }
    }
};

// MARK: - ReassemblyManager

/// Reconstructs complete messages from incoming fragments.
/// @note NOT thread-safe. Must be used from a single thread (typically the receive loop).
///       The returned buffer from ProcessFragment is valid until the next call.
class ReassemblyManager {
public:
    using Clock = std::chrono::steady_clock;

    ReassemblyManager() = default;
    ~ReassemblyManager() = default;

    // Non-copyable
    ReassemblyManager(const ReassemblyManager&) = delete;
    ReassemblyManager& operator=(const ReassemblyManager&) = delete;

    /// Process an incoming UDP packet that may be a fragment
    /// @param data Raw UDP packet data
    /// @param size Size of UDP packet
    /// @param out_message_size Set to complete message size if reassembly complete
    /// @return Pointer to complete message buffer if reassembly complete, nullptr otherwise
    /// @note The returned pointer is valid until the next call to ProcessFragment
    const uint8_t* ProcessFragment(const uint8_t* data, size_t size,
                                   size_t* out_message_size);

    /// Clean up stale incomplete messages (call periodically)
    void CleanupStale();

    /// Get number of active reassembly slots
    int GetActiveSlotCount() const;

    /// Reset all reassembly state
    void Reset();

private:
    std::array<ReassemblySlot, MJ_REASSEMBLY_SLOTS> slots_;

    /// Find existing slot for message_id or allocate a new one
    /// Uses LRU eviction if all slots are full
    ReassemblySlot* FindOrCreateSlot(uint16_t message_id);
};

} // namespace imujoco

#endif // __cplusplus

#endif /* mjc_fragment_h */
