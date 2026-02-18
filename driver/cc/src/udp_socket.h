// udp_socket.h
// Simple POSIX UDP socket wrapper (macOS/Linux)

#ifndef IMUJOCO_DRIVER_UDP_SOCKET_H
#define IMUJOCO_DRIVER_UDP_SOCKET_H

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <span>
#include <string>

namespace imujoco::driver {

/// Simple UDP socket wrapper with blocking operations and timeout support.
class UdpSocket {
public:
    UdpSocket() = default;

    ~UdpSocket() {
        Close();
    }

    // Non-copyable
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Movable
    UdpSocket(UdpSocket&& other) noexcept
        : socket_(other.socket_),
          remote_addr_(other.remote_addr_),
          remote_addr_len_(other.remote_addr_len_) {
        other.socket_ = -1;
        other.remote_addr_len_ = 0;
    }

    UdpSocket& operator=(UdpSocket&& other) noexcept {
        if (this != &other) {
            Close();
            socket_ = other.socket_;
            remote_addr_ = other.remote_addr_;
            remote_addr_len_ = other.remote_addr_len_;
            other.socket_ = -1;
            other.remote_addr_len_ = 0;
        }
        return *this;
    }

    /// Initialize the socket and bind to local port.
    /// @param local_port Local port to bind (0 = ephemeral)
    /// @return true if successful
    bool Initialize(uint16_t local_port = 0) {
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ < 0) {
            return false;
        }

        // Increase socket buffers to reduce packet loss under burst traffic
        int rcvbuf = 2 * 1024 * 1024;  // 2 MB receive buffer
        int sndbuf = 1 * 1024 * 1024;  // 1 MB send buffer
        if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
            // Non-fatal: OS may cap the value; proceed with default buffer size
        }
        if (setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
            // Non-fatal: OS may cap the value; proceed with default buffer size
        }

        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(local_port);

        if (bind(socket_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
            Close();
            return false;
        }

        return true;
    }

    /// Set the remote endpoint to send to.
    /// @param host Remote host (IP or hostname)
    /// @param port Remote port
    /// @return true if successful
    bool SetRemote(const std::string& host, uint16_t port) {
        remote_addr_ = {};
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(port);

        // Try to parse as IP address first
        if (inet_pton(AF_INET, host.c_str(), &remote_addr_.sin_addr) == 1) {
            remote_addr_len_ = sizeof(remote_addr_);
            return true;
        }

        // Try DNS resolution
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;

        addrinfo* result = nullptr;
        if (getaddrinfo(host.c_str(), nullptr, &hints, &result) != 0 || !result) {
            return false;
        }

        // Find first IPv4 address in results
        bool found = false;
        for (addrinfo* rp = result; rp != nullptr; rp = rp->ai_next) {
            if (rp->ai_family == AF_INET && rp->ai_addr) {
                auto* addr = reinterpret_cast<sockaddr_in*>(rp->ai_addr);
                remote_addr_.sin_addr = addr->sin_addr;
                remote_addr_len_ = sizeof(remote_addr_);
                found = true;
                break;
            }
        }

        freeaddrinfo(result);
        return found;
    }

    /// Send data to the remote endpoint.
    /// @param data Data to send
    /// @return Number of bytes sent, or -1 on error
    ssize_t Send(std::span<const uint8_t> data) {
        if (socket_ < 0 || remote_addr_len_ == 0) {
            return -1;
        }

        return sendto(socket_, data.data(), data.size(), 0,
                      reinterpret_cast<const sockaddr*>(&remote_addr_), remote_addr_len_);
    }

    /// Receive data with timeout.
    /// @param buffer Buffer to receive into
    /// @param timeout_ms Timeout in milliseconds (0 = no timeout)
    /// @return Number of bytes received, 0 on timeout, or -1 on error
    ssize_t Receive(std::span<uint8_t> buffer, uint32_t timeout_ms = 0) {
        if (socket_ < 0) {
            return -1;
        }

        if (timeout_ms > 0) {
            pollfd pfd{};
            pfd.fd = socket_;
            pfd.events = POLLIN;

            int result = poll(&pfd, 1, static_cast<int>(timeout_ms));
            if (result == 0) {
                return 0;  // Timeout
            }
            if (result < 0) {
                return -1;  // Error
            }
        }

        sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);

        return recvfrom(socket_, buffer.data(), buffer.size(), 0,
                        reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);
    }

    /// Close the socket.
    void Close() {
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
    }

    /// Check if socket is open.
    bool IsOpen() const {
        return socket_ >= 0;
    }

private:
    int socket_ = -1;
    sockaddr_in remote_addr_{};
    socklen_t remote_addr_len_ = 0;
};

}  // namespace imujoco::driver

#endif  // IMUJOCO_DRIVER_UDP_SOCKET_H
