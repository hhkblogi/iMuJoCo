// mjc_fragment.mm
// UDP packet fragmentation and reassembly implementation

#include "mjc_fragment.h"
#include <os/log.h>
#include <sys/socket.h>
#include <algorithm>
#include <cstring>

// MARK: - Logging

static os_log_t GetFragmentLog() {
    static os_log_t log = os_log_create("com.imujoco.core", "fragment");
    return log;
}

// MARK: - CRC-16 Implementation

// CRC-16-CCITT lookup table (polynomial 0x1021)
static const uint16_t kCRC16Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t mj_fragment_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ kCRC16Table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

uint16_t mj_fragment_header_checksum(const MJFragmentHeader* header) {
    // Compute CRC over header bytes excluding the checksum field (last 2 bytes)
    return mj_fragment_crc16(reinterpret_cast<const uint8_t*>(header),
                             MJ_FRAGMENT_HEADER_SIZE - sizeof(uint16_t));
}

bool mj_fragment_header_valid(const MJFragmentHeader* header) {
    if (!header) return false;
    if (header->magic != MJ_PACKET_MAGIC_FRAG) return false;
    return header->checksum == mj_fragment_header_checksum(header);
}

// MARK: - FragmentedSender Implementation

namespace imujoco {

int FragmentedSender::SendMessage(const uint8_t* data, size_t size,
                                   int socket_fd, const struct sockaddr_in& dest) {
    if (!data || size == 0) {
        os_log_error(GetFragmentLog(), "SendMessage: invalid data or size");
        return -1;
    }

    if (size > MJ_MAX_MESSAGE_SIZE) {
        os_log_error(GetFragmentLog(), "SendMessage: message too large: %zu > %zu",
                     size, static_cast<size_t>(MJ_MAX_MESSAGE_SIZE));
        return -1;
    }

    // Calculate number of fragments needed
    // Static assert ensures MJ_MAX_MESSAGE_SIZE is exactly representable with uint8_t fragment count
    static_assert(MJ_MAX_MESSAGE_SIZE == MJ_MAX_FRAGMENT_PAYLOAD * MJ_MAX_FRAGMENTS,
                  "MJ_MAX_MESSAGE_SIZE must equal MJ_MAX_FRAGMENT_PAYLOAD * MJ_MAX_FRAGMENTS");
    uint8_t fragment_count = static_cast<uint8_t>(
        (size + MJ_MAX_FRAGMENT_PAYLOAD - 1) / MJ_MAX_FRAGMENT_PAYLOAD);

    // Get unique message ID
    uint16_t msg_id = message_id_.fetch_add(1, std::memory_order_relaxed);

    // Ensure fragment buffer is large enough
    fragment_buffer_.resize(MJ_MAX_UDP_PAYLOAD);

    size_t offset = 0;
    for (uint8_t i = 0; i < fragment_count; i++) {
        size_t payload_size = std::min(static_cast<size_t>(MJ_MAX_FRAGMENT_PAYLOAD),
                                       size - offset);

        // Build fragment header
        MJFragmentHeader* header = reinterpret_cast<MJFragmentHeader*>(fragment_buffer_.data());
        header->magic = MJ_PACKET_MAGIC_FRAG;
        header->message_id = msg_id;
        header->fragment_index = i;
        header->fragment_count = fragment_count;
        header->total_size = static_cast<uint32_t>(size);
        header->payload_size = static_cast<uint16_t>(payload_size);
        header->checksum = 0;
        header->checksum = mj_fragment_header_checksum(header);

        // Copy payload
        memcpy(fragment_buffer_.data() + MJ_FRAGMENT_HEADER_SIZE,
               data + offset, payload_size);

        // Send fragment
        size_t packet_size = MJ_FRAGMENT_HEADER_SIZE + payload_size;
        ssize_t sent = sendto(socket_fd, fragment_buffer_.data(), packet_size, 0,
                              reinterpret_cast<const struct sockaddr*>(&dest),
                              sizeof(dest));

        if (sent != static_cast<ssize_t>(packet_size)) {
            os_log_error(GetFragmentLog(),
                         "SendMessage: sendto failed for fragment %u/%u: %d",
                         i + 1, fragment_count, errno);
            return -1;
        }

        offset += payload_size;
    }

    os_log_debug(GetFragmentLog(),
                 "SendMessage: sent %u fragments, msg_id=%u, total_size=%zu",
                 fragment_count, msg_id, size);

    return fragment_count;
}

// MARK: - ReassemblyManager Implementation

const uint8_t* ReassemblyManager::ProcessFragment(const uint8_t* data, size_t size,
                                                   size_t* out_message_size) {
    if (!data || !out_message_size) {
        return nullptr;
    }

    *out_message_size = 0;

    // Validate minimum size
    if (size < MJ_FRAGMENT_HEADER_SIZE) {
        return nullptr;
    }

    const MJFragmentHeader* header = reinterpret_cast<const MJFragmentHeader*>(data);

    // Validate magic
    if (header->magic != MJ_PACKET_MAGIC_FRAG) {
        return nullptr;
    }

    // Validate checksum
    if (!mj_fragment_header_valid(header)) {
        os_log_error(GetFragmentLog(), "ProcessFragment: checksum mismatch for msg_id=%u",
                     header->message_id);
        return nullptr;
    }

    // Validate header fields
    if (header->fragment_index >= header->fragment_count ||
        header->fragment_count == 0 ||
        header->payload_size > MJ_MAX_FRAGMENT_PAYLOAD ||
        header->payload_size + MJ_FRAGMENT_HEADER_SIZE != size ||
        header->total_size > MJ_MAX_MESSAGE_SIZE) {
        os_log_error(GetFragmentLog(),
                     "ProcessFragment: invalid header fields - idx=%u count=%u payload=%u size=%zu",
                     header->fragment_index, header->fragment_count,
                     header->payload_size, size);
        return nullptr;
    }

    // Validate fragment_count is consistent with total_size
    uint8_t expected_count = static_cast<uint8_t>(
        (header->total_size + MJ_MAX_FRAGMENT_PAYLOAD - 1) / MJ_MAX_FRAGMENT_PAYLOAD);
    if (header->fragment_count != expected_count) {
        os_log_error(GetFragmentLog(),
                     "ProcessFragment: fragment_count mismatch - got=%u expected=%u for total_size=%u",
                     header->fragment_count, expected_count, header->total_size);
        return nullptr;
    }

    // Find or create reassembly slot
    ReassemblySlot* slot = FindOrCreateSlot(header->message_id);
    if (!slot) {
        os_log_error(GetFragmentLog(), "ProcessFragment: no available slots");
        return nullptr;
    }

    // Initialize slot if new
    if (!slot->active) {
        slot->message_id = header->message_id;
        slot->fragment_count = header->fragment_count;
        slot->total_size = header->total_size;
        slot->received_count = 0;
        slot->received_mask = 0;
        memset(slot->extended_mask, 0, sizeof(slot->extended_mask));
        slot->buffer.resize(header->total_size);
        slot->active = true;
    }

    // Validate consistency with existing slot
    if (slot->fragment_count != header->fragment_count ||
        slot->total_size != header->total_size) {
        os_log_error(GetFragmentLog(),
                     "ProcessFragment: inconsistent fragment for msg_id=%u",
                     header->message_id);
        return nullptr;
    }

    // Check for duplicate fragment
    if (slot->IsFragmentReceived(header->fragment_index)) {
        // Duplicate - update activity time but don't process again
        slot->last_activity = Clock::now();
        return nullptr;
    }

    // Mark fragment as received
    slot->MarkFragmentReceived(header->fragment_index);
    slot->received_count++;
    slot->last_activity = Clock::now();

    // Copy payload to correct position in buffer
    size_t offset = static_cast<size_t>(header->fragment_index) * MJ_MAX_FRAGMENT_PAYLOAD;
    const uint8_t* payload = data + MJ_FRAGMENT_HEADER_SIZE;

    // Validate that payload fits within total message buffer (security check)
    if (offset + header->payload_size > slot->total_size) {
        os_log_error(GetFragmentLog(),
                     "ProcessFragment: payload exceeds buffer - offset=%zu payload=%u total=%u",
                     offset, header->payload_size, slot->total_size);
        return nullptr;
    }

    memcpy(slot->buffer.data() + offset, payload, header->payload_size);

    os_log_debug(GetFragmentLog(),
                 "ProcessFragment: received %u/%u for msg_id=%u",
                 slot->received_count, slot->fragment_count, slot->message_id);

    // Check if complete
    if (slot->received_count == slot->fragment_count) {
        *out_message_size = slot->total_size;

        os_log_info(GetFragmentLog(),
                    "ProcessFragment: reassembled msg_id=%u (%u bytes, %u fragments)",
                    slot->message_id, slot->total_size, slot->fragment_count);

        // Mark slot as inactive to prevent message_id collision on wrap-around.
        // Buffer contents remain valid until caller copies or slot is reused.
        slot->active = false;

        return slot->buffer.data();
    }

    return nullptr;
}

ReassemblySlot* ReassemblyManager::FindOrCreateSlot(uint16_t message_id) {
    Clock::time_point now = Clock::now();
    ReassemblySlot* oldest_slot = nullptr;
    Clock::time_point oldest_time = Clock::time_point::max();
    ReassemblySlot* empty_slot = nullptr;

    for (auto& slot : slots_) {
        // Found existing slot for this message
        if (slot.active && slot.message_id == message_id) {
            return &slot;
        }

        // Track empty slot
        if (!slot.active && !empty_slot) {
            empty_slot = &slot;
        }

        // Track oldest for LRU eviction
        if (slot.active && slot.last_activity < oldest_time) {
            oldest_time = slot.last_activity;
            oldest_slot = &slot;
        }
    }

    // Return empty slot if available
    if (empty_slot) {
        empty_slot->last_activity = now;
        return empty_slot;
    }

    // Evict oldest (LRU)
    if (oldest_slot) {
        os_log_debug(GetFragmentLog(),
                     "FindOrCreateSlot: evicting stale msg_id=%u (%u/%u received)",
                     oldest_slot->message_id, oldest_slot->received_count,
                     oldest_slot->fragment_count);
        oldest_slot->Reset();
        oldest_slot->last_activity = now;
        return oldest_slot;
    }

    return nullptr;
}

void ReassemblyManager::CleanupStale() {
    auto now = Clock::now();
    auto timeout = std::chrono::milliseconds(MJ_REASSEMBLY_TIMEOUT_MS);

    for (auto& slot : slots_) {
        if (slot.active && (now - slot.last_activity) > timeout) {
            os_log_debug(GetFragmentLog(),
                         "CleanupStale: timing out msg_id=%u (%u/%u received)",
                         slot.message_id, slot.received_count, slot.fragment_count);
            slot.Reset();
        }
    }
}

int ReassemblyManager::GetActiveSlotCount() const {
    int count = 0;
    for (const auto& slot : slots_) {
        if (slot.active) count++;
    }
    return count;
}

void ReassemblyManager::Reset() {
    for (auto& slot : slots_) {
        slot.Reset();
    }
}

} // namespace imujoco
