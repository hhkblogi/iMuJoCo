// time_sync.cc
// gPTP-inspired clock synchronization implementation.

#include "imujoco/driver/time_sync.h"
#include "mjc_sync_protocol.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

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

        // Build and send request
        uint64_t t1 = NowUs();
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

        // Wait for response — drain stale responses to find current seq.
        // A single WiFi hiccup can put us one-behind permanently if we don't drain.
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        bool matched = false;
        bool received_any = false;  // Any valid packet (even stale seq)
        int poll_result = poll(&pfd, 1, static_cast<int>(config_.timeout_ms));
        while (poll_result > 0) {
            struct sockaddr_in from_addr;
            socklen_t from_len = sizeof(from_addr);
            ssize_t recv_len = recvfrom(socket_fd_, recv_buf, sizeof(recv_buf), 0,
                                        reinterpret_cast<struct sockaddr*>(&from_addr),
                                        &from_len);

            uint64_t t4 = NowUs();

            if (recv_len < 0) {
                recv_error_count++;
            } else if (recv_len >= static_cast<ssize_t>(sizeof(imujoco::protocol::MJPdelayResponse))) {
                auto* resp = reinterpret_cast<const imujoco::protocol::MJPdelayResponse*>(recv_buf);
                if (!imujoco::protocol::mj_sync_validate_response(resp)) {
                    validate_fail_count++;
                } else if (resp->seq != req.seq) {
                    seq_mismatch_count++;
                    received_any = true;
                } else {
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
                }
            }

            // Non-blocking check for more stale data
            poll_result = poll(&pfd, 1, 0);
        }
        if (poll_result == 0 && !matched) {
            poll_timeout_count++;

            // Only count true no-data timeouts toward recovery threshold.
            // Stale-seq responses mean the link is alive, just laggy.
            if (!received_any) {
                consecutive_timeouts++;
            }

            // If we've timed out too many times and the servo was locked,
            // reset to allow clean reconvergence when connectivity returns.
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
