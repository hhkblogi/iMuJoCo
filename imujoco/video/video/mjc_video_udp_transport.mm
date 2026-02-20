// mjc_video_udp_transport.mm
// Raw UDP video transport implementation
// Uses FragmentedSender from mjc_fragment.h for MTU-safe delivery

#include "mjc_video_udp_transport.h"
#include "imujoco/core/core/mjc_fragment.h"

#include <cstring>
#include <memory>

// Network includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

// Apple unified logging
#include <os/log.h>

// MARK: - Construction / Destruction

MJVideoUDPTransport::MJVideoUDPTransport()
    : socket_fd_(-1),
      port_(0),
      active_(false),
      has_receiver_(false),
      sender_(std::make_unique<imujoco::FragmentedSender>()) {
    std::memset(&receiver_addr_, 0, sizeof(receiver_addr_));
}

MJVideoUDPTransport::~MJVideoUDPTransport() {
    Stop();
}

// MARK: - Start / Stop

bool MJVideoUDPTransport::Start(uint16_t port) {
    if (active_.load(std::memory_order_acquire)) {
        return true;  // Already started
    }

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        os_log_error(OS_LOG_DEFAULT, "VideoUDP: failed to create socket");
        return false;
    }

    // Non-blocking for PollForReceiver
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    int opt = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Increase send buffer for video data (256KB)
    int sndbuf = 256 * 1024;
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        os_log_error(OS_LOG_DEFAULT, "VideoUDP: failed to bind to port %u", port);
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    port_ = port;
    active_.store(true, std::memory_order_release);
    os_log_info(OS_LOG_DEFAULT, "VideoUDP: listening on port %u", port);
    return true;
}

void MJVideoUDPTransport::Stop() {
    active_.store(false, std::memory_order_release);
    has_receiver_.store(false, std::memory_order_release);

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        os_log_info(OS_LOG_DEFAULT, "VideoUDP: closed port %u", port_);
    }
    port_ = 0;
}

// MARK: - SendFrame

bool MJVideoUDPTransport::SendFrame(const MJVideoFrameDesc& desc,
                                     const uint8_t* data, size_t size) {
    if (!active_.load(std::memory_order_acquire) || socket_fd_ < 0) {
        return false;
    }

    // Check for new/changed receiver
    PollForReceiver();

    if (!has_receiver_.load(std::memory_order_acquire)) {
        return false;  // Nobody listening — skip silently
    }

    // Build message: [MJVideoFrameDesc (40 bytes)][pixel data]
    size_t total = sizeof(MJVideoFrameDesc) + size;
    send_buffer_.resize(total);
    std::memcpy(send_buffer_.data(), &desc, sizeof(MJVideoFrameDesc));
    std::memcpy(send_buffer_.data() + sizeof(MJVideoFrameDesc), data, size);

    int fragments = sender_->SendMessage(
        send_buffer_.data(), total, socket_fd_, receiver_addr_);

    if (fragments < 0) {
        os_log_error(OS_LOG_DEFAULT, "VideoUDP: SendMessage failed for frame %llu",
                     desc.frame_number);
        return false;
    }

    return true;
}

// MARK: - Receiver Discovery

void MJVideoUDPTransport::PollForReceiver() {
    // Non-blocking recv to check for "hello" packets from receivers.
    // Any packet (even 1 byte) registers the sender as the receiver.
    uint8_t buf[64];
    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    ssize_t n = recvfrom(socket_fd_, buf, sizeof(buf), 0,
                         reinterpret_cast<struct sockaddr*>(&sender_addr), &addr_len);

    if (n > 0) {
        // Check if receiver changed
        if (!has_receiver_.load(std::memory_order_acquire) ||
            sender_addr.sin_addr.s_addr != receiver_addr_.sin_addr.s_addr ||
            sender_addr.sin_port != receiver_addr_.sin_port) {

            receiver_addr_ = sender_addr;
            has_receiver_.store(true, std::memory_order_release);

            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &sender_addr.sin_addr, ip_str, sizeof(ip_str));
            os_log_info(OS_LOG_DEFAULT, "VideoUDP: receiver registered %{public}s:%u",
                        ip_str, ntohs(sender_addr.sin_port));
        }
    }
}

// MARK: - Status

bool MJVideoUDPTransport::HasReceiver() const {
    return has_receiver_.load(std::memory_order_acquire);
}

bool MJVideoUDPTransport::IsActive() const {
    return active_.load(std::memory_order_acquire);
}
