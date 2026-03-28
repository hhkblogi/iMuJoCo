// time_sync.cc
// gPTP-inspired clock synchronization implementation.

#include "imujoco/driver/time_sync.h"
#include "mjc_sync_protocol.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sys/time.h>

// Network includes
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace imujoco::driver {

using Clock = std::chrono::steady_clock;

static uint64_t NowUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            Clock::now().time_since_epoch()).count());
}

/// Wall-clock microseconds (gettimeofday). Same clock domain as SO_TIMESTAMP
/// kernel receive timestamps, enabling accurate t4 for late-arriving responses.
static uint64_t WallUs() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<uint64_t>(tv.tv_sec) * 1'000'000 +
           static_cast<uint64_t>(tv.tv_usec);
}

// ============================================================================
// PTPClockServo
// ============================================================================

void PTPClockServo::ProcessExchange(const PdelayExchange& ex) {
    // Compute raw offset and delay from peer-delay exchange
    // offset = ((t2-t1) + (t3-t4)) / 2   (device clock - driver clock)
    // delay  = ((t2-t1) - (t3-t4)) / 2   (one-way propagation delay)
    int64_t forward  = static_cast<int64_t>(ex.t2) - static_cast<int64_t>(ex.t1);
    int64_t backward = static_cast<int64_t>(ex.t3) - static_cast<int64_t>(ex.t4);
    int64_t raw_offset = (forward + backward) / 2;
    int64_t raw_delay  = (forward - backward) / 2;

    if (raw_delay < 0) raw_delay = 0;  // Clamp negative delay

    // Reject exchanges with unreasonable delay (>50ms one-way).
    // Safety net for corrupted timestamps or extreme outliers.
    if (raw_delay > 50'000) return;

    // --- Min-RTT filter (NTP clock filter algorithm) ---
    // WiFi delay is asymmetric: upstream ≠ downstream queuing.
    // The sample with minimum RTT has the least total queuing, so its
    // offset is closest to truth. Select it from a sliding window.
    int64_t rtt = forward - backward;  // = 2 * one_way_delay
    rtt_window_.push_back({raw_offset, raw_delay, rtt});
    if (static_cast<int>(rtt_window_.size()) > config_.min_rtt_window) {
        rtt_window_.pop_front();
    }
    if (rtt_window_.size() >= 2) {
        auto best = std::min_element(rtt_window_.begin(), rtt_window_.end(),
            [](const RttSample& a, const RttSample& b) { return a.rtt < b.rtt; });
        raw_offset = best->raw_offset;
        raw_delay = best->raw_delay;
    }

    exchanges_++;

    // --- Outlier rejection (Welford online variance, 3-sigma) ---
    count_++;
    double delta = static_cast<double>(raw_offset) - mean_;
    mean_ += delta / count_;
    double delta2 = static_cast<double>(raw_offset) - mean_;
    m2_ += delta * delta2;

    if (count_ > config_.min_samples_for_lock) {
        double variance = m2_ / (count_ - 1);
        double stddev = std::sqrt(variance);
        double deviation = std::abs(static_cast<double>(raw_offset) - mean_);
        if (deviation > config_.outlier_threshold * stddev && stddev > 0.0) {
            // Outlier — discard but don't undo Welford update
            outliers_rejected_++;
            return;
        }
    }

    // --- Median filter ---
    median_buffer_.push_back(raw_offset);
    if (static_cast<int>(median_buffer_.size()) > config_.median_window) {
        median_buffer_.pop_front();
    }

    int64_t filtered_offset;
    {
        std::vector<int64_t> sorted(median_buffer_.begin(), median_buffer_.end());
        std::sort(sorted.begin(), sorted.end());
        filtered_offset = sorted[sorted.size() / 2];
    }

    // --- 3-phase state machine (linuxptp-style) ---
    delay_us_ = raw_delay;

    if (phase_count_ == 0) {
        // Phase 0: Direct step to first filtered value (no PI)
        offset_us_ = filtered_offset;
        phase_count_ = 1;

        lock_window_.push_back(offset_us_);
        if (static_cast<int>(lock_window_.size()) > config_.lock_window_size) {
            lock_window_.pop_front();
        }
        return;
    }

    if (phase_count_ == 1) {
        // Phase 1: Refine step, reset integral for clean PI start
        offset_us_ = filtered_offset;
        integral_ = 0.0;
        phase_count_ = 2;

        lock_window_.push_back(offset_us_);
        if (static_cast<int>(lock_window_.size()) > config_.lock_window_size) {
            lock_window_.pop_front();
        }
        return;
    }

    // Phase 2+: PI controller with anti-windup
    double error = static_cast<double>(filtered_offset) - static_cast<double>(offset_us_);
    double correction = config_.kp * error + config_.ki * integral_;

    if (std::abs(correction) <= config_.max_correction_us) {
        // Unclamped — accumulate integral
        integral_ += error;
    } else {
        // Clamped — freeze integral (anti-windup)
        correction = (correction > 0) ? config_.max_correction_us
                                       : -config_.max_correction_us;
    }
    offset_us_ += static_cast<int64_t>(correction);

    // --- Lock detection (linuxptp offset-threshold, servo.c:100-114) ---
    // Still push to lock_window for jitter diagnostics
    lock_window_.push_back(offset_us_);
    if (static_cast<int>(lock_window_.size()) > config_.lock_window_size) {
        lock_window_.pop_front();
    }

    // Offset-threshold: |PI error| < threshold for N consecutive samples
    double error_mag = std::abs(error);
    if (exchanges_ >= config_.min_samples_for_lock) {
        if (error_mag < config_.lock_threshold_us) {
            if (stable_count_ > 0) stable_count_--;
        } else {
            stable_count_ = config_.num_offset_values;  // reset counter
        }
        bool was_locked = locked_;
        locked_ = (stable_count_ == 0);
        if (locked_ && !was_locked) lock_count_++;
        if (!locked_ && was_locked) unlock_count_++;
    }
}

void PTPClockServo::UpdateRateRatio(int32_t rate_ratio_ppm) {
    rate_ratio_ppm_ = rate_ratio_ppm;
}

ClockSyncState PTPClockServo::GetState() const {
    ClockSyncState state;
    state.locked = locked_;
    state.offset_us = offset_us_;
    state.delay_us = delay_us_;
    state.rate_ratio_ppm = rate_ratio_ppm_;
    state.exchanges = exchanges_;
    state.raw_jitter_us = (count_ > 1) ? std::sqrt(m2_ / (count_ - 1)) : 0.0;
    // Windowed jitter on PI output (what lock uses)
    if (lock_window_.size() >= 2) {
        double sum = 0.0;
        for (auto v : lock_window_) sum += static_cast<double>(v);
        double win_mean = sum / static_cast<double>(lock_window_.size());
        double sq_sum = 0.0;
        for (auto v : lock_window_) {
            double d = static_cast<double>(v) - win_mean;
            sq_sum += d * d;
        }
        state.jitter_us = std::sqrt(sq_sum / static_cast<double>(lock_window_.size() - 1));
    } else {
        state.jitter_us = 0.0;
    }
    state.outliers_rejected = outliers_rejected_;
    state.pi_integral = integral_;
    state.mean_offset_us = mean_;
    state.lock_count = lock_count_;
    state.unlock_count = unlock_count_;
    return state;
}

uint64_t PTPClockServo::DriverToDeviceTime(uint64_t driver_us) const {
    return static_cast<uint64_t>(static_cast<int64_t>(driver_us) + offset_us_);
}

uint64_t PTPClockServo::DeviceToDriverTime(uint64_t device_us) const {
    return static_cast<uint64_t>(static_cast<int64_t>(device_us) - offset_us_);
}

void PTPClockServo::Reset() {
    count_ = 0;
    mean_ = 0.0;
    m2_ = 0.0;
    rtt_window_.clear();
    median_buffer_.clear();
    lock_window_.clear();
    integral_ = 0.0;
    offset_us_ = 0;
    delay_us_ = 0;
    rate_ratio_ppm_ = 0;
    exchanges_ = 0;
    locked_ = false;
    phase_count_ = 0;
    stable_count_ = config_.num_offset_values;
    outliers_rejected_ = 0;
    lock_count_ = 0;
    unlock_count_ = 0;
}

// ============================================================================
// SimTimeClock
// ============================================================================

void SimTimeClock::UpdateAnchor(const SimTimeAnchor& anchor) {
    if (anchor.device_wall_us == 0) return;
    anchor_ = anchor;
    has_anchor_ = true;
}

std::optional<double> SimTimeClock::PredictSimTime(
    uint64_t driver_now_us, const PTPClockServo& servo) const {
    if (!has_anchor_ || anchor_.timestep_us == 0) return std::nullopt;

    // Convert driver time to device time
    uint64_t device_now_us = servo.DriverToDeviceTime(driver_now_us);

    // Extrapolate from anchor
    int64_t elapsed_us = static_cast<int64_t>(device_now_us) -
                         static_cast<int64_t>(anchor_.device_wall_us);
    double elapsed_s = static_cast<double>(elapsed_us) / 1e6;

    return anchor_.sim_time + elapsed_s;
}

void SimTimeClock::Reset() {
    has_anchor_ = false;
    anchor_ = {};
}

// ============================================================================
// SyncClient
// ============================================================================

SyncClient::SyncClient(PTPClockServo& servo, std::mutex& servo_mutex,
                       const Config& config)
    : config_(config), servo_(servo), servo_mutex_(servo_mutex) {}

SyncClient::~SyncClient() {
    Stop();
}

bool SyncClient::Start() {
    if (running_.load(std::memory_order_acquire)) return true;

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd_ < 0) return false;

    // Allow rapid socket reuse after Stop()
    int opt = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Enable kernel receive timestamps. SO_TIMESTAMP uses the wall clock
    // (gettimeofday), same domain as WallUs() used for t1. This gives
    // accurate t4 even for prev-seq responses read one cycle late.
    setsockopt(socket_fd_, SOL_SOCKET, SO_TIMESTAMP, &opt, sizeof(opt));

    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&SyncClient::sync_thread_func, this);
    return true;
}

void SyncClient::Stop() {
    if (!running_.load(std::memory_order_acquire)) return;
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

void SyncClient::sync_thread_func() {
    // Resolve target address
    struct sockaddr_in target_addr;
    std::memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(config_.port);
    if (inet_pton(AF_INET, config_.host.c_str(), &target_addr.sin_addr) != 1) {
        running_.store(false, std::memory_order_release);
        return;
    }

    uint8_t recv_buf[64];  // Enough for MJPdelayResponse (42 bytes)
    char cmsg_buf[CMSG_SPACE(sizeof(struct timeval))];

    // Diagnostic counters
    uint64_t send_count = 0;
    uint64_t recv_count = 0;
    uint64_t poll_timeout_count = 0;
    uint64_t poll_error_count = 0;
    uint64_t send_error_count = 0;
    uint64_t recv_error_count = 0;
    uint64_t seq_mismatch_count = 0;
    uint64_t validate_fail_count = 0;
    auto last_log_time = Clock::now();
    uint32_t consecutive_timeouts = 0;

    while (running_.load(std::memory_order_acquire)) {
        auto loop_start = Clock::now();

        // Build and send request (wall clock for t1, matches SO_TIMESTAMP for t4)
        uint64_t t1 = WallUs();
        imujoco::protocol::MJPdelayRequest req;
        req.seq = seq_++;
        req.t1_us = t1;
        imujoco::protocol::mj_sync_build_request(&req);

        ssize_t send_result = sendto(socket_fd_, &req, sizeof(req), 0,
               reinterpret_cast<const struct sockaddr*>(&target_addr),
               sizeof(target_addr));
        if (send_result > 0) {
            send_count++;
        } else {
            send_error_count++;
        }

        // Poll for responses. Drain all available packets, preferring
        // current seq but falling back to prev-seq if no exact match.
        // Prev-seq responses have inflated t4 (~50ms extra delay) since
        // we read them one cycle late, but they still prove connectivity
        // and keep the servo fed when current-seq consistently misses.
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        bool matched = false;
        bool received_any = false;
        // Stash best prev-seq response in case current seq never arrives
        bool have_prev = false;
        PdelayExchange prev_exchange;
        int32_t prev_rate_ratio = 0;

        int poll_result = poll(&pfd, 1, static_cast<int>(config_.timeout_ms));
        while (poll_result > 0) {
            // Use recvmsg to extract kernel receive timestamp (SO_TIMESTAMP)
            struct sockaddr_in from_addr;
            struct iovec iov = { recv_buf, sizeof(recv_buf) };
            struct msghdr msg = {};
            msg.msg_name = &from_addr;
            msg.msg_namelen = sizeof(from_addr);
            msg.msg_iov = &iov;
            msg.msg_iovlen = 1;
            msg.msg_control = cmsg_buf;
            msg.msg_controllen = sizeof(cmsg_buf);

            ssize_t recv_len = recvmsg(socket_fd_, &msg, 0);
            uint64_t t4 = WallUs();  // fallback
            for (struct cmsghdr* cm = CMSG_FIRSTHDR(&msg);
                 cm != nullptr; cm = CMSG_NXTHDR(&msg, cm)) {
                if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_TIMESTAMP) {
                    auto* tv = reinterpret_cast<struct timeval*>(CMSG_DATA(cm));
                    t4 = static_cast<uint64_t>(tv->tv_sec) * 1'000'000 +
                         static_cast<uint64_t>(tv->tv_usec);
                    break;
                }
            }

            if (recv_len < 0) {
                recv_error_count++;
            } else if (recv_len >= static_cast<ssize_t>(sizeof(imujoco::protocol::MJPdelayResponse))) {
                auto* resp = reinterpret_cast<const imujoco::protocol::MJPdelayResponse*>(recv_buf);
                if (!imujoco::protocol::mj_sync_validate_response(resp)) {
                    validate_fail_count++;
                } else if (resp->seq == req.seq) {
                    // Exact match — use immediately
                    recv_count++;
                    matched = true;
                    PdelayExchange exchange;
                    exchange.t1 = resp->t1_us;
                    exchange.t2 = resp->t2_us;
                    exchange.t3 = resp->t3_us;
                    exchange.t4 = t4;

                    std::lock_guard<std::mutex> lock(servo_mutex_);
                    servo_.ProcessExchange(exchange);
                    servo_.UpdateRateRatio(resp->rate_ratio_ppm);

                    // Send sync feedback to device every 10th exchange
                    if (recv_count % 10 == 0) {
                        auto state = servo_.GetState();
                        imujoco::protocol::MJSyncFeedback fb;
                        fb.offset_us = state.offset_us;
                        fb.delay_us = state.delay_us;
                        fb.jitter_us = static_cast<float>(state.jitter_us);
                        fb.locked = state.locked ? 1 : 0;
                        fb.exchanges = state.exchanges;
                        imujoco::protocol::mj_sync_build_feedback(&fb);
                        sendto(socket_fd_, &fb, sizeof(fb), 0,
                               reinterpret_cast<const struct sockaddr*>(&target_addr),
                               sizeof(target_addr));
                    }

                    consecutive_timeouts = 0;
                    break;  // Got our match, done
                } else if (req.seq > 0 && resp->seq == req.seq - 1) {
                    // Previous seq — stash as fallback. SO_TIMESTAMP gives
                    // accurate kernel t4 even though we read it late.
                    have_prev = true;
                    prev_exchange = {resp->t1_us, resp->t2_us, resp->t3_us, t4};
                    prev_rate_ratio = resp->rate_ratio_ppm;
                    received_any = true;
                } else {
                    seq_mismatch_count++;
                    received_any = true;
                }
            }

            // Non-blocking check for more data
            poll_result = poll(&pfd, 1, 0);
        }

        // If no current-seq match, use prev-seq fallback
        if (!matched && have_prev) {
            recv_count++;
            matched = true;
            std::lock_guard<std::mutex> lock(servo_mutex_);
            servo_.ProcessExchange(prev_exchange);
            servo_.UpdateRateRatio(prev_rate_ratio);
            consecutive_timeouts = 0;
        }
        if (poll_result == 0 && !matched) {
            poll_timeout_count++;

            if (!received_any) {
                consecutive_timeouts++;
            }

            if (config_.max_consecutive_timeouts > 0 &&
                consecutive_timeouts >= config_.max_consecutive_timeouts) {
                std::lock_guard<std::mutex> lock(servo_mutex_);
                if (servo_.GetState().locked) {
                    fprintf(stderr,
                        "[SyncClient] %u consecutive timeouts while locked — resetting servo\n",
                        consecutive_timeouts);
                    servo_.Reset();
                }
                consecutive_timeouts = 0;
            }
        } else if (poll_result < 0) {
            poll_error_count++;
        }

        // Log diagnostics every 60s
        auto now = Clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 60) {
            fprintf(stderr, "[SyncClient] sent=%llu recv=%llu poll_timeout=%llu "
                    "poll_err=%llu send_err=%llu recv_err=%llu seq_mismatch=%llu validate_fail=%llu\n",
                    send_count, recv_count, poll_timeout_count,
                    poll_error_count, send_error_count, recv_error_count,
                    seq_mismatch_count, validate_fail_count);
            last_log_time = now;
        }

        // Maintain cadence
        auto elapsed = Clock::now() - loop_start;
        auto target = std::chrono::milliseconds(config_.interval_ms);
        if (elapsed < target) {
            std::this_thread::sleep_for(target - elapsed);
        }
    }
}

}  // namespace imujoco::driver
