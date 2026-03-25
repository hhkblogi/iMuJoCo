// mjc_sync_server.h
// gPTP-inspired sync responder thread for iMuJoCo device (master role).
//
// Runs on a dedicated thread, owns its own UDP socket on ctrl_port + 100.
// Responds to MJPdelayRequest with MJPdelayResponse including rate ratio.
// Independent of the physics loop — no shared mutable state.

#ifndef mjc_sync_server_h
#define mjc_sync_server_h

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

// Network includes
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

// Apple unified logging
#include <os/log.h>

#include "mjc_sync_protocol.h"

class SyncServer {
public:
    SyncServer() = default;
    ~SyncServer() { Stop(); }

    // Non-copyable
    SyncServer(const SyncServer&) = delete;
    SyncServer& operator=(const SyncServer&) = delete;

    /// Start the sync responder on a dedicated thread.
    /// @param port UDP port to listen on
    /// @param bind_address Local address to bind to
    /// @return true if socket opened and thread launched
    bool Start(uint16_t port, const std::string& bind_address) {
        if (running_.load(std::memory_order_acquire)) return true;

        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            os_log_error(OS_LOG_DEFAULT, "SyncServer: failed to create socket");
            return false;
        }

        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr);
        addr.sin_port = htons(port);

        if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            os_log_error(OS_LOG_DEFAULT, "SyncServer: failed to bind to %{public}s:%u",
                         bind_address.c_str(), port);
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        port_ = port;
        running_.store(true, std::memory_order_release);
        thread_ = std::thread(&SyncServer::RunLoop, this);

        os_log_info(OS_LOG_DEFAULT, "SyncServer: listening on %{public}s:%u",
                    bind_address.c_str(), port);
        return true;
    }

    /// Stop the sync responder thread and close the socket.
    void Stop() {
        if (!running_.load(std::memory_order_acquire)) return;

        running_.store(false, std::memory_order_release);
        if (thread_.joinable()) {
            thread_.join();
        }

        responses_sent_.store(0, std::memory_order_relaxed);

        // Reset rate-ratio state so a restart doesn't use stale deltas
        prev_t1_us_ = 0;
        prev_t2_us_ = 0;
        smoothed_rate_ratio_ppm_ = 0.0;
        rate_ratio_ppm_ = 0;
        atomic_rate_ratio_ppm_.store(0, std::memory_order_relaxed);

        // Reset driver feedback atomics
        driver_offset_us_.store(0, std::memory_order_relaxed);
        driver_delay_us_.store(0, std::memory_order_relaxed);
        driver_jitter_us_milli_.store(0, std::memory_order_relaxed);
        driver_locked_.store(false, std::memory_order_relaxed);
        driver_exchanges_.store(0, std::memory_order_relaxed);

        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
            os_log_info(OS_LOG_DEFAULT, "SyncServer: closed port %u", port_);
        }
    }

    bool IsRunning() const { return running_.load(std::memory_order_acquire); }
    uint16_t GetPort() const { return port_; }
    uint64_t GetResponsesSent() const { return responses_sent_.load(std::memory_order_relaxed); }
    int32_t GetRateRatioPpm() const { return atomic_rate_ratio_ppm_.load(std::memory_order_relaxed); }

    // Driver-side feedback (received via MJSyncFeedback messages)
    int64_t GetDriverOffsetUs() const { return driver_offset_us_.load(std::memory_order_relaxed); }
    int64_t GetDriverDelayUs() const { return driver_delay_us_.load(std::memory_order_relaxed); }
    float GetDriverJitterUs() const { return static_cast<float>(driver_jitter_us_milli_.load(std::memory_order_relaxed)) / 1000.0f; }
    bool GetDriverLocked() const { return driver_locked_.load(std::memory_order_relaxed); }
    uint32_t GetDriverExchanges() const { return driver_exchanges_.load(std::memory_order_relaxed); }

private:
    using Clock = std::chrono::steady_clock;

    /// Get current steady_clock time in microseconds.
    static uint64_t NowUs() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                Clock::now().time_since_epoch()).count());
    }

    /// Main responder loop — runs on dedicated thread.
    void RunLoop() {
        uint8_t buffer[64];  // Enough for MJPdelayRequest (18 bytes)
        uint64_t poll_errors = 0;
        uint64_t recv_errors = 0;
        uint64_t last_feedback_us = 0;
        uint64_t last_response_us = 0;
        bool feedback_stale = true;  // Start stale until first feedback arrives

        while (running_.load(std::memory_order_acquire)) {
            // Poll with 100ms timeout to allow clean shutdown
            struct pollfd pfd;
            pfd.fd = socket_fd_;
            pfd.events = POLLIN;

            int poll_result = poll(&pfd, 1, 100);
            if (poll_result < 0) {
                poll_errors++;
                if (poll_errors <= 5 || (poll_errors % 100) == 0) {
                    os_log_error(OS_LOG_DEFAULT,
                        "SyncServer: poll error %d (errno=%d, total=%llu, sent=%llu)",
                        poll_result, errno, poll_errors, responses_sent_.load(std::memory_order_relaxed));
                }
                continue;
            }
            if (poll_result == 0) {
                // Check for stale driver connection (no activity for 5s).
                // Use whichever timestamp is more recent: feedback or response.
                uint64_t last_activity = last_feedback_us > last_response_us
                                       ? last_feedback_us : last_response_us;
                if (!feedback_stale && last_activity > 0) {
                    uint64_t now = NowUs();
                    if (now - last_activity > 5'000'000) {
                        driver_offset_us_.store(0, std::memory_order_relaxed);
                        driver_delay_us_.store(0, std::memory_order_relaxed);
                        driver_jitter_us_milli_.store(0, std::memory_order_relaxed);
                        driver_locked_.store(false, std::memory_order_relaxed);
                        driver_exchanges_.store(0, std::memory_order_relaxed);
                        feedback_stale = true;
                        os_log_info(OS_LOG_DEFAULT, "SyncServer: driver feedback stale, cleared metrics");
                    }
                }
                continue;
            }

            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);

            ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                        reinterpret_cast<struct sockaddr*>(&client_addr),
                                        &addr_len);

            // Capture receive timestamp immediately
            uint64_t t2 = NowUs();

            if (recv_len < 0) {
                recv_errors++;
                if (recv_errors <= 5 || (recv_errors % 100) == 0) {
                    os_log_error(OS_LOG_DEFAULT,
                        "SyncServer: recvfrom error (errno=%d, total=%llu, sent=%llu)",
                        errno, recv_errors, responses_sent_.load(std::memory_order_relaxed));
                }
                continue;
            }

            // Check magic to dispatch: feedback vs pdelay request
            if (recv_len >= static_cast<ssize_t>(sizeof(imujoco::protocol::MJSyncFeedback))) {
                auto* fb = reinterpret_cast<const imujoco::protocol::MJSyncFeedback*>(buffer);
                if (imujoco::protocol::mj_sync_validate_feedback(fb)) {
                    driver_offset_us_.store(fb->offset_us, std::memory_order_relaxed);
                    driver_delay_us_.store(fb->delay_us, std::memory_order_relaxed);
                    driver_jitter_us_milli_.store(static_cast<int32_t>(fb->jitter_us * 1000.0f), std::memory_order_relaxed);
                    driver_locked_.store(fb->locked != 0, std::memory_order_relaxed);
                    driver_exchanges_.store(fb->exchanges, std::memory_order_relaxed);
                    last_feedback_us = t2;
                    feedback_stale = false;
                    continue;
                }
            }

            if (recv_len < static_cast<ssize_t>(sizeof(imujoco::protocol::MJPdelayRequest))) {
                continue;
            }

            auto* req = reinterpret_cast<const imujoco::protocol::MJPdelayRequest*>(buffer);
            if (!imujoco::protocol::mj_sync_validate_request(req)) {
                continue;
            }

            // Update rate ratio from consecutive (t1, t2) pairs
            uint64_t t1 = req->t1_us;
            if (prev_t1_us_ > 0 && t1 > prev_t1_us_ && t2 > prev_t2_us_) {
                double driver_delta = static_cast<double>(t1 - prev_t1_us_);
                double device_delta = static_cast<double>(t2 - prev_t2_us_);
                if (driver_delta > 0.0) {
                    double ratio = (device_delta / driver_delta - 1.0) * 1e6;
                    // EMA smoothing (alpha = 0.1)
                    smoothed_rate_ratio_ppm_ = smoothed_rate_ratio_ppm_ * 0.9 + ratio * 0.1;
                    rate_ratio_ppm_ = static_cast<int32_t>(smoothed_rate_ratio_ppm_);
                    atomic_rate_ratio_ppm_.store(rate_ratio_ppm_, std::memory_order_relaxed);
                }
            }
            prev_t1_us_ = t1;
            prev_t2_us_ = t2;

            // Build response
            uint64_t t3 = NowUs();
            imujoco::protocol::MJPdelayResponse resp;
            resp.seq = req->seq;
            resp.t1_us = req->t1_us;
            resp.t2_us = t2;
            resp.t3_us = t3;
            resp.rate_ratio_ppm = rate_ratio_ppm_;
            imujoco::protocol::mj_sync_build_response(&resp);

            ssize_t sent = sendto(socket_fd_, &resp, sizeof(resp), 0,
                   reinterpret_cast<struct sockaddr*>(&client_addr), addr_len);
            if (sent > 0) {
                last_response_us = NowUs();
                feedback_stale = false;  // A driver is connected
                auto count = responses_sent_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (count == 1 || (count % 600) == 0) {
                    os_log_info(OS_LOG_DEFAULT,
                        "SyncServer: sent=%llu poll_err=%llu recv_err=%llu",
                        count, poll_errors, recv_errors);
                }
            }
        }

        os_log_info(OS_LOG_DEFAULT,
            "SyncServer: RunLoop exiting (sent=%llu, poll_err=%llu, recv_err=%llu)",
            responses_sent_.load(std::memory_order_relaxed), poll_errors, recv_errors);
    }

    int socket_fd_ = -1;
    uint16_t port_ = 0;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> responses_sent_{0};
    std::atomic<int32_t> atomic_rate_ratio_ppm_{0};
    std::thread thread_;

    // Driver-side feedback atomics (written from RunLoop, read from UI thread)
    std::atomic<int64_t> driver_offset_us_{0};
    std::atomic<int64_t> driver_delay_us_{0};
    std::atomic<int32_t> driver_jitter_us_milli_{0};
    std::atomic<bool> driver_locked_{false};
    std::atomic<uint32_t> driver_exchanges_{0};

    // Rate ratio state (accessed only from RunLoop thread)
    uint64_t prev_t1_us_ = 0;
    uint64_t prev_t2_us_ = 0;
    double smoothed_rate_ratio_ppm_ = 0.0;
    int32_t rate_ratio_ppm_ = 0;
};

#endif  // mjc_sync_server_h
