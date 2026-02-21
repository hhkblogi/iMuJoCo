// mjc_video_types.h
// Video streaming types: frame descriptor, format enum, configuration
//
// MJVideoFrameDesc is a fixed 40-byte header prepended to each video frame
// payload. It uses native byte order (same-architecture communication) and
// is NOT a FlatBuffers message — it's kept simple for the high-bandwidth
// video path where every microsecond counts.

#ifndef IMUJOCO_VIDEO_TYPES_H_
#define IMUJOCO_VIDEO_TYPES_H_

#include <cstdint>

// MARK: - Video Format Enum

/// Pixel format of the video frame payload.
enum class MJVideoFormat : uint8_t {
    RGBA8    = 0,  ///< 4 bytes/pixel, uncompressed
    RGB8     = 1,  ///< 3 bytes/pixel, uncompressed
    DEPTH32F = 2,  ///< 4 bytes/pixel, 32-bit float depth
    JPEG     = 3,  ///< Variable length, JPEG-compressed
};

// MARK: - Video Frame Descriptor (40 bytes, packed)

/// Fixed-size header prepended to video frame data.
/// Sent as [MJVideoFrameDesc][pixel data] through the transport.
///
/// The frame_number field is monotonically increasing and can be used
/// to detect gaps. simulation_time correlates with StatePacket.time
/// for video-state synchronization.
#pragma pack(push, 1)
struct MJVideoFrameDesc {
    uint32_t width;             ///< Frame width in pixels
    uint32_t height;            ///< Frame height in pixels
    uint32_t stride;            ///< Bytes per row (may include padding)
    uint8_t  format;            ///< MJVideoFormat (RGBA8, RGB8, DEPTH32F, JPEG)
    uint8_t  camera_index;      ///< Index of the camera in the model
    uint8_t  reserved[2];       ///< Padding for alignment
    double   simulation_time;   ///< Simulation time when frame was captured (matches StatePacket.time)
    uint64_t frame_number;      ///< Monotonically increasing frame counter
    uint32_t data_size;         ///< Size of pixel data following this header
    uint32_t checksum;          ///< CRC-32 of the pixel data
};
#pragma pack(pop)

static_assert(sizeof(MJVideoFrameDesc) == 40, "MJVideoFrameDesc must be exactly 40 bytes");

// MARK: - Video Configuration

/// Configuration for a single camera stream.
struct MJVideoCameraConfig {
    uint8_t  camera_index = 0;  ///< Model camera index
    uint32_t width = 256;       ///< Capture width
    uint32_t height = 256;      ///< Capture height
    uint8_t  format = 0;        ///< MJVideoFormat (default RGBA8)
};

/// Overall video streaming configuration.
struct MJVideoConfig {
    bool     enabled = false;       ///< Master enable/disable
    float    target_fps = 30.0f;    ///< Target capture FPS (independent of display FPS)
    uint16_t port = 0;              ///< Video port (0 = control_port + 100)
    float    jpeg_quality = 0.8f;   ///< JPEG quality (0.0–1.0, used when format=JPEG)
};

// MARK: - Video Port Offset

/// Default offset from control port to video port.
constexpr uint16_t MJ_VIDEO_PORT_OFFSET = 100;

// MARK: - CRC-32 for video frames

#ifdef __cplusplus

#include <cstddef>

/// Compute CRC-32 checksum for video frame data.
/// Uses the standard CRC-32 polynomial (same as zlib/Ethernet).
inline uint32_t mjc_video_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    return ~crc;
}

#endif  // __cplusplus

#endif  // IMUJOCO_VIDEO_TYPES_H_
