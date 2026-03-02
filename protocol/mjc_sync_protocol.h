// mjc_sync_protocol.h
// gPTP-inspired time synchronization wire protocol for iMuJoCo
//
// Packed C structs with CRC-16-CCITT integrity checking.
// Used by both device (core) and driver for peer-delay exchanges.
// Self-contained: no external dependencies beyond <cstdint> / <cstring>.

#pragma once

#include <cstdint>
#include <cstring>

namespace imujoco::protocol {

// ============================================================================
// CRC-16-CCITT (0xFFFF initial, poly 0x1021)
// ============================================================================

namespace detail {

// Pre-computed CRC-16-CCITT lookup table
static constexpr uint16_t kCrc16Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x4864, 0x5845, 0x6826, 0x7807, 0x08E0, 0x18C1, 0x28A2, 0x38A3,
    0xC94C, 0xD96D, 0xE90E, 0xF92F, 0x89C8, 0x99E9, 0xA98A, 0xB9AB,
    0x5A75, 0x4A54, 0x7A37, 0x6A16, 0x1AF1, 0x0AD0, 0x3AB3, 0x2A92,
    0xDB7D, 0xCB5C, 0xFB3F, 0xEB1E, 0x9BF9, 0x8BD8, 0xBBBB, 0xAB9A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x04A1, 0x7446, 0x6467, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

}  // namespace detail

/// Compute CRC-16-CCITT over a byte range.
inline uint16_t mj_sync_crc16(const void* data, size_t len) {
    uint16_t crc = 0xFFFF;
    const auto* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        crc = (crc << 8) ^ detail::kCrc16Table[((crc >> 8) ^ p[i]) & 0xFF];
    }
    return crc;
}

// ============================================================================
// Wire protocol constants
// ============================================================================

static constexpr uint16_t MJ_SYNC_PORT        = 9000;        // Fixed sync port for all instances
static constexpr uint32_t MJ_SYNC_MAGIC_REQ  = 0x4D4A5351;  // "MJSQ"
static constexpr uint32_t MJ_SYNC_MAGIC_RESP = 0x4D4A5352;  // "MJSR"

// ============================================================================
// Packed structures (no padding)
// ============================================================================

#pragma pack(push, 1)

/// Peer-delay request: follower → master (18 bytes)
struct MJPdelayRequest {
    uint32_t magic;       // MJ_SYNC_MAGIC_REQ
    uint32_t seq;         // Exchange sequence number
    uint64_t t1_us;       // Follower send timestamp (steady_clock microseconds)
    uint16_t checksum;    // CRC-16 over preceding fields
};

/// Peer-delay response: master → follower (42 bytes)
struct MJPdelayResponse {
    uint32_t magic;             // MJ_SYNC_MAGIC_RESP
    uint32_t seq;               // Echo of request seq
    uint64_t t1_us;             // Echo of request t1
    uint64_t t2_us;             // Master recv timestamp (steady_clock microseconds)
    uint64_t t3_us;             // Master send timestamp (steady_clock microseconds)
    int32_t  rate_ratio_ppm;    // (f_master/f_follower - 1) * 1e6
    uint16_t checksum;          // CRC-16 over preceding fields
};

#pragma pack(pop)

// Compile-time size checks
static_assert(sizeof(MJPdelayRequest)  == 18, "MJPdelayRequest must be 18 bytes");
static_assert(sizeof(MJPdelayResponse) == 38, "MJPdelayResponse must be 38 bytes");

// ============================================================================
// Validation helpers
// ============================================================================

/// Validate a received PdelayRequest (magic + checksum).
inline bool mj_sync_validate_request(const MJPdelayRequest* req) {
    if (!req) return false;
    if (req->magic != MJ_SYNC_MAGIC_REQ) return false;
    uint16_t expected = mj_sync_crc16(req, sizeof(*req) - sizeof(req->checksum));
    return req->checksum == expected;
}

/// Validate a received PdelayResponse (magic + checksum).
inline bool mj_sync_validate_response(const MJPdelayResponse* resp) {
    if (!resp) return false;
    if (resp->magic != MJ_SYNC_MAGIC_RESP) return false;
    uint16_t expected = mj_sync_crc16(resp, sizeof(*resp) - sizeof(resp->checksum));
    return resp->checksum == expected;
}

// ============================================================================
// Build helpers (set magic + compute checksum)
// ============================================================================

/// Finalize a PdelayRequest: set magic and compute checksum.
/// Caller must fill seq, t1_us before calling.
inline void mj_sync_build_request(MJPdelayRequest* req) {
    req->magic = MJ_SYNC_MAGIC_REQ;
    req->checksum = mj_sync_crc16(req, sizeof(*req) - sizeof(req->checksum));
}

/// Finalize a PdelayResponse: set magic and compute checksum.
/// Caller must fill seq, t1_us, t2_us, t3_us, rate_ratio_ppm before calling.
inline void mj_sync_build_response(MJPdelayResponse* resp) {
    resp->magic = MJ_SYNC_MAGIC_RESP;
    resp->checksum = mj_sync_crc16(resp, sizeof(*resp) - sizeof(resp->checksum));
}

}  // namespace imujoco::protocol
