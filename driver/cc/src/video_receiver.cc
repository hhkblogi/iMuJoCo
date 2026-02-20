// video_receiver.cc
// Video receiver implementation — RX thread with fragment reassembly

#include "imujoco/driver/video_receiver.h"
#include "fragment.h"

#include <chrono>
#include <cstring>

// Network includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

namespace imujoco::driver {

// Wire format of MJVideoFrameDesc (must match sim side exactly)
#pragma pack(push, 1)
struct WireVideoFrameDesc {
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint8_t  format;
    uint8_t  camera_index;
    uint8_t  reserved[2];
    double   simulation_time;
    uint64_t frame_number;
    uint32_t data_size;
    uint32_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(WireVideoFrameDesc) == 40, "Wire format must be 40 bytes");

// CRC-32 (same algorithm as sim side)
static uint32_t compute_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    return ~crc;
}

// ============================================================================
// Construction / Destruction
// ============================================================================

VideoReceiver::VideoReceiver(const VideoReceiverConfig& config)
    : config_(config),
      socket_fd_(-1),
      reassembler_(std::make_unique<ReassemblyManager>()) {
    recv_buffer_.resize(kMaxUDPPayload);
    std::memset(&remote_addr_, 0, sizeof(remote_addr_));
}

VideoReceiver::~VideoReceiver() {
    Stop();
}

// ============================================================================
// Start / Stop
// ============================================================================

bool VideoReceiver::Start(VideoFrameCallback callback) {
    if (running_.load(std::memory_order_acquire)) {
        return true;
    }

    // Create and bind socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        return false;
    }

    int opt = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Increase receive buffer for video data (512KB)
    int rcvbuf = 512 * 1024;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in bind_addr;
    std::memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port = htons(config_.local_port);

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&bind_addr),
             sizeof(bind_addr)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set up remote address for hello packets
    std::memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(config_.port);
    inet_pton(AF_INET, config_.host.c_str(), &remote_addr_.sin_addr);

    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback_ = std::move(callback);
    }

    running_.store(true, std::memory_order_release);
    rx_thread_ = std::thread(&VideoReceiver::rx_thread_func, this);
    return true;
}

void VideoReceiver::Stop() {
    running_.store(false, std::memory_order_release);

    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback_ = nullptr;
    }
}

bool VideoReceiver::IsRunning() const {
    return running_.load(std::memory_order_acquire);
}

// ============================================================================
// RX Thread
// ============================================================================

void VideoReceiver::rx_thread_func() {
    using Clock = std::chrono::steady_clock;
    auto last_hello = Clock::now() - std::chrono::seconds(10);  // Send immediately
    uint32_t cleanup_counter = 0;

    while (running_.load(std::memory_order_acquire)) {
        // Periodically send hello to register with sim
        auto now = Clock::now();
        auto since_hello = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_hello);
        if (since_hello.count() >= static_cast<int64_t>(config_.hello_interval_ms)) {
            send_hello();
            last_hello = now;
        }

        // Poll for data with timeout
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        int timeout_ms = config_.timeout_ms > 0
            ? static_cast<int>(config_.timeout_ms)
            : 50;  // Default 50ms to allow hello re-sends

        int ret = poll(&pfd, 1, timeout_ms);
        if (ret <= 0) continue;  // Timeout or error

        // Read packet
        struct sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);

        ssize_t n = recvfrom(socket_fd_, recv_buffer_.data(), recv_buffer_.size(), 0,
                             reinterpret_cast<struct sockaddr*>(&sender_addr), &addr_len);

        if (n <= 0) continue;

        stat_fragments_received_.fetch_add(1, std::memory_order_relaxed);

        // Throttle stale cleanup
        if (++cleanup_counter % 64 == 0) {
            reassembler_->CleanupStale();
        }

        // Check for fragment header
        if (n >= static_cast<ssize_t>(kFragmentHeaderSize)) {
            auto* frag = reinterpret_cast<const FragmentHeader*>(recv_buffer_.data());
            if (frag->magic == kFragmentMagic) {
                auto result = reassembler_->ProcessFragment(
                    recv_buffer_.data(), static_cast<size_t>(n));

                if (result.complete) {
                    parse_frame(result.data, result.size);
                }
                continue;
            }
        }

        // Non-fragmented packet (small frames)
        parse_frame(recv_buffer_.data(), static_cast<size_t>(n));
    }
}

void VideoReceiver::send_hello() {
    // Send a 1-byte "hello" to register with the sim's video port
    uint8_t hello = 0x01;
    sendto(socket_fd_, &hello, 1, 0,
           reinterpret_cast<const struct sockaddr*>(&remote_addr_),
           sizeof(remote_addr_));
}

bool VideoReceiver::parse_frame(const uint8_t* data, size_t size) {
    if (size < sizeof(WireVideoFrameDesc)) {
        stat_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    const auto* wire = reinterpret_cast<const WireVideoFrameDesc*>(data);
    const uint8_t* pixel_data = data + sizeof(WireVideoFrameDesc);
    size_t pixel_size = size - sizeof(WireVideoFrameDesc);

    // Validate data_size matches
    if (wire->data_size != pixel_size) {
        stat_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    // Validate CRC-32
    uint32_t computed_crc = compute_crc32(pixel_data, pixel_size);
    if (computed_crc != wire->checksum) {
        stat_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    // Build VideoFrame
    VideoFrame frame;
    frame.desc.width = wire->width;
    frame.desc.height = wire->height;
    frame.desc.stride = wire->stride;
    frame.desc.format = static_cast<VideoFormat>(wire->format);
    frame.desc.camera_index = wire->camera_index;
    frame.desc.simulation_time = wire->simulation_time;
    frame.desc.frame_number = wire->frame_number;
    frame.desc.data_size = wire->data_size;
    frame.desc.checksum = wire->checksum;
    frame.data.assign(pixel_data, pixel_data + pixel_size);

    stat_frames_received_.fetch_add(1, std::memory_order_relaxed);
    stat_bytes_received_.fetch_add(size, std::memory_order_relaxed);
    stat_last_sim_time_.store(wire->simulation_time, std::memory_order_relaxed);

    // Dispatch to callback
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
        callback_(frame);
    }

    return true;
}

// ============================================================================
// Statistics
// ============================================================================

VideoReceiverStats VideoReceiver::GetStats() const {
    VideoReceiverStats stats;
    stats.frames_received = stat_frames_received_.load(std::memory_order_relaxed);
    stats.frames_dropped = stat_frames_dropped_.load(std::memory_order_relaxed);
    stats.fragments_received = stat_fragments_received_.load(std::memory_order_relaxed);
    stats.bytes_received = stat_bytes_received_.load(std::memory_order_relaxed);
    stats.last_simulation_time = stat_last_sim_time_.load(std::memory_order_relaxed);
    return stats;
}

void VideoReceiver::ResetStats() {
    stat_frames_received_.store(0, std::memory_order_relaxed);
    stat_frames_dropped_.store(0, std::memory_order_relaxed);
    stat_fragments_received_.store(0, std::memory_order_relaxed);
    stat_bytes_received_.store(0, std::memory_order_relaxed);
    stat_last_sim_time_.store(0.0, std::memory_order_relaxed);
}

}  // namespace imujoco::driver
