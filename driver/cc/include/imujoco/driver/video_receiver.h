// video_receiver.h
// Receives video frames from iMuJoCo simulation via UDP
//
// Architecture:
//   - Sends a "hello" packet to the simulation's video port to register
//   - Dedicated RX thread reassembles fragmented UDP packets
//   - Delivers complete frames via callback (runs on RX thread)
//   - Thread-safe public API

#ifndef IMUJOCO_DRIVER_VIDEO_RECEIVER_H
#define IMUJOCO_DRIVER_VIDEO_RECEIVER_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <thread>
#include <vector>

namespace imujoco::driver {

// Forward declarations
class ReassemblyManager;

/// Video pixel format (mirrors MJVideoFormat on the sim side).
enum class VideoFormat : uint8_t {
    RGBA8    = 0,
    RGB8     = 1,
    DEPTH32F = 2,
    JPEG     = 3,
};

/// Frame descriptor received with each video frame.
/// This is a host-side copy of MJVideoFrameDesc (40 bytes on wire).
struct VideoFrameDesc {
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    VideoFormat format;
    uint8_t  camera_index;
    double   simulation_time;
    uint64_t frame_number;
    uint32_t data_size;
    uint32_t checksum;
};

/// A received video frame: descriptor + pixel data.
struct VideoFrame {
    VideoFrameDesc desc;
    std::vector<uint8_t> data;  ///< Pixel data (owned copy)
};

/// Callback invoked on the RX thread for each received frame.
/// Keep callbacks lightweight or dispatch to your own thread.
using VideoFrameCallback = std::function<void(const VideoFrame&)>;

/// Configuration for the video receiver.
struct VideoReceiverConfig {
    /// Target simulation host (default: localhost)
    std::string host = "127.0.0.1";

    /// Target simulation video port
    uint16_t port = 9100;

    /// Local bind port (0 = ephemeral)
    uint16_t local_port = 0;

    /// Receive timeout in milliseconds (0 = no timeout)
    uint32_t timeout_ms = 100;

    /// Hello packet interval in milliseconds (re-register with sim)
    uint32_t hello_interval_ms = 1000;
};

/// Video receiver statistics.
struct VideoReceiverStats {
    uint64_t frames_received = 0;
    uint64_t frames_dropped = 0;     ///< CRC mismatch or reassembly failure
    uint64_t fragments_received = 0;
    uint64_t bytes_received = 0;
    double   last_simulation_time = 0.0;
};

/// Receives video frames from an iMuJoCo simulation over UDP.
///
/// Usage:
/// ```cpp
/// VideoReceiver rx({.host = "192.168.1.5", .port = 9100});
/// rx.Start([](const VideoFrame& frame) {
///     printf("Frame %llu: %ux%u\n", frame.desc.frame_number,
///            frame.desc.width, frame.desc.height);
/// });
/// // ... later ...
/// rx.Stop();
/// ```
class VideoReceiver {
public:
    explicit VideoReceiver(const VideoReceiverConfig& config = {});
    ~VideoReceiver();

    // Non-copyable, non-movable
    VideoReceiver(const VideoReceiver&) = delete;
    VideoReceiver& operator=(const VideoReceiver&) = delete;

    /// Start receiving video frames.
    /// @param callback Called on RX thread for each complete frame
    /// @return true if started successfully
    bool Start(VideoFrameCallback callback);

    /// Stop receiving.
    void Stop();

    /// Check if receiving.
    bool IsRunning() const;

    /// Get current statistics.
    VideoReceiverStats GetStats() const;

    /// Reset statistics to zero.
    void ResetStats();

private:
    void rx_thread_func();
    void send_hello();
    bool parse_frame(const uint8_t* data, size_t size);

    VideoReceiverConfig config_;

    // Socket
    int socket_fd_;
    struct sockaddr_in remote_addr_;

    // RX thread
    std::thread rx_thread_;
    std::atomic<bool> running_{false};

    // Reassembly
    std::unique_ptr<ReassemblyManager> reassembler_;
    std::vector<uint8_t> recv_buffer_;

    // Callback
    std::mutex callback_mutex_;
    VideoFrameCallback callback_;

    // Statistics
    std::atomic<uint64_t> stat_frames_received_{0};
    std::atomic<uint64_t> stat_frames_dropped_{0};
    std::atomic<uint64_t> stat_fragments_received_{0};
    std::atomic<uint64_t> stat_bytes_received_{0};
    std::atomic<double> stat_last_sim_time_{0.0};
};

}  // namespace imujoco::driver

#endif  // IMUJOCO_DRIVER_VIDEO_RECEIVER_H
