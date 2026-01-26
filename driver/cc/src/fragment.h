// fragment.h
// UDP packet fragmentation and reassembly for FlatBuffers messages
// Cross-platform port of imujoco/core/mjc_fragment.h

#ifndef IMUJOCO_DRIVER_FRAGMENT_H
#define IMUJOCO_DRIVER_FRAGMENT_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace imujoco::driver {

// Network constants
// NOTE: These values assume IPv4 with no IP options (20-byte header) and 8-byte
// UDP header. For IPv6 (40-byte header) or IPv4 with options, constants would
// need adjustment. All multi-byte header fields use native byte order (no
// network byte order conversion). This protocol is intended for
// same-architecture local network communication.
constexpr size_t kMTU = 1500;
constexpr size_t kIPUDPOverhead = 28;  // 20 bytes IP + 8 bytes UDP
constexpr size_t kMaxUDPPayload =
    1472;  // MTU - IP/UDP overhead (space for header + payload)
constexpr size_t kFragmentHeaderSize = 16;
constexpr size_t kMaxFragmentPayload =
    1456;  // Max UDP payload - fragment header
constexpr uint8_t kMaxFragments = 255;
constexpr size_t kMaxMessageSize =
    kMaxFragments * kMaxFragmentPayload;  // ~371KB

// Fragment packet magic number: "MJFG" (MuJoCo Fragment)
constexpr uint32_t kFragmentMagic = 0x4D4A4647;

// Reassembly configuration
constexpr size_t kReassemblySlots = 8;       // Max concurrent incomplete messages
constexpr int kReassemblyTimeoutMs = 100;    // Timeout for incomplete messages

// Fragment header structure (16 bytes, packed)
// Placed at the start of each UDP packet containing a fragment
#pragma pack(push, 1)
struct FragmentHeader {
    uint32_t magic;           // kFragmentMagic (0x4D4A4647)
    uint16_t message_id;      // Unique ID correlating fragments of same message
    uint8_t fragment_index;   // 0-based position in sequence (0-254)
    uint8_t fragment_count;   // Total number of fragments (1-255)
    uint32_t total_size;      // Total size of complete message payload
    uint16_t payload_size;    // Size of payload in this fragment
    uint16_t checksum;        // CRC-16 of header (excluding checksum field)
};
#pragma pack(pop)

static_assert(sizeof(FragmentHeader) == 16, "FragmentHeader must be 16 bytes");

// CRC-16 utilities
uint16_t ComputeCRC16(const uint8_t* data, size_t length);
uint16_t ComputeHeaderChecksum(const FragmentHeader* header);
bool ValidateHeaderChecksum(const FragmentHeader* header);

// Splits large messages into MTU-sized fragments
// NOT thread-safe - each thread should use its own instance
class FragmentedSender {
public:
    FragmentedSender() = default;
    ~FragmentedSender() = default;

    // Non-copyable
    FragmentedSender(const FragmentedSender&) = delete;
    FragmentedSender& operator=(const FragmentedSender&) = delete;

    // Fragment a message into UDP packets
    // Returns vector of fragments, each ready to send
    // Each fragment includes the FragmentHeader followed by payload
    std::vector<std::vector<uint8_t>> FragmentMessage(const uint8_t* data,
                                                       size_t size);

    // Get the next message ID that will be used
    uint16_t NextMessageId() const { return message_id_; }

private:
    uint16_t message_id_ = 0;
};

// Internal structure for tracking incomplete message reassembly
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
        std::memset(extended_mask, 0, sizeof(extended_mask));
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

// Result of processing a fragment
struct ReassemblyResult {
    bool complete = false;           // True if message is complete
    const uint8_t* data = nullptr;   // Pointer to complete message (valid until next call)
    size_t size = 0;                 // Size of complete message
};

// Reconstructs complete messages from incoming fragments
// NOT thread-safe - must be used from a single thread
class ReassemblyManager {
public:
    using Clock = std::chrono::steady_clock;

    ReassemblyManager() = default;
    ~ReassemblyManager() = default;

    // Non-copyable
    ReassemblyManager(const ReassemblyManager&) = delete;
    ReassemblyManager& operator=(const ReassemblyManager&) = delete;

    // Process an incoming UDP packet that may be a fragment
    // Returns result indicating if reassembly is complete
    // The returned data pointer is valid until the next call to ProcessFragment
    ReassemblyResult ProcessFragment(const uint8_t* data, size_t size);

    // Clean up stale incomplete messages (call periodically)
    void CleanupStale();

    // Get number of active reassembly slots
    int ActiveSlotCount() const;

    // Reset all reassembly state
    void Reset();

private:
    std::array<ReassemblySlot, kReassemblySlots> slots_;

    // Find existing slot for message_id or allocate a new one
    // Uses LRU eviction if all slots are full
    ReassemblySlot* find_or_create_slot(uint16_t message_id);
};

}  // namespace imujoco::driver

#endif  // IMUJOCO_DRIVER_FRAGMENT_H
