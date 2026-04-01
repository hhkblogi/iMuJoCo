// time_sync.h
// gPTP-inspired clock synchronization for iMuJoCo driver (follower role).
//
// Components:
//   PTPClockServo — PI controller with median filter + outlier rejection
//   SimTimeClock  — maps driver wall-clock to predicted simulation time
//   SyncClient    — dedicated UDP thread for peer-delay exchanges

#ifndef IMUJOCO_DRIVER_TIME_SYNC_H
#define IMUJOCO_DRIVER_TIME_SYNC_H

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace imujoco::driver {

// ============================================================================
// Types
// ============================================================================

/// A single peer-delay exchange (all times in microseconds).
struct PdelayExchange {
    uint64_t t1;  // Follower send time
    uint64_t t2;  // Master recv time
    uint64_t t3;  // Master send time
    uint64_t t4;  // Follower recv time
};

/// Snapshot of clock synchronization state.
struct ClockSyncState {
    bool locked = false;           // True when servo has converged
    int64_t offset_us = 0;         // Device = driver + offset
    int64_t delay_us = 0;          // One-way delay estimate
    int32_t rate_ratio_ppm = 0;    // (f_device/f_driver - 1) * 1e6
    int exchanges = 0;             // Total exchanges processed
    double jitter_us = 0.0;       // Windowed PI output jitter (used for lock)
    double raw_jitter_us = 0.0;   // Full-history Welford jitter (diagnostic)
    int outliers_rejected = 0;     // Total outlier samples rejected
    double pi_integral = 0.0;      // PI controller integral term
    double mean_offset_us = 0.0;   // Welford running mean offset
    int lock_count = 0;            // Number of times lock was achieved
    int unlock_count = 0;          // Number of times lock was lost
};

/// Anchor point for simulation time prediction.
struct SimTimeAnchor {
    double sim_time = 0.0;          // Simulation time at anchor
    uint64_t device_wall_us = 0;    // Device wall-clock at anchor
    uint64_t step_index = 0;        // Step index at anchor
    uint32_t timestep_us = 0;       // Model timestep in microseconds
};

// ============================================================================
// PTPClockServo
// ============================================================================

/// PI clock servo with min-RTT filter, median filter, and outlier rejection.
/// Computes the offset between driver and device clocks from peer-delay
/// exchanges.
class PTPClockServo {
public:
    struct Config {
        double kp = 0.20;                   // PI proportional gain: linuxptp 0.1 × 0.1^(-0.3) at 10Hz
        double ki = 0.0004;                 // PI integral gain: linuxptp 0.001 × 0.1^(0.4) at 10Hz
        int median_window = 7;              // Median filter window (odd)
        double outlier_threshold = 3.0;     // 3-sigma outlier rejection
        int min_samples_for_lock = 10;      // Samples before declaring lock
        double lock_threshold_us = 1000.0;  // Offset-threshold for lock (µs)
        int lock_window_size = 30;          // Lock window for jitter diagnostics (3s at 10Hz)
        double max_correction_us = 1000.0;  // Anti-windup clamp (µs)
        int num_offset_values = 10;         // Consecutive below-threshold for LOCKED_STABLE
        int min_rtt_window = 8;             // Min-RTT filter window (NTP-style, ~0.8s at 10Hz)
    };

    PTPClockServo() = default;
    explicit PTPClockServo(const Config& config)
        : config_(config), stable_count_(config.num_offset_values) {}

    /// Process a new peer-delay exchange.
    void ProcessExchange(const PdelayExchange& ex);

    /// Update rate ratio from device-reported value.
    void UpdateRateRatio(int32_t rate_ratio_ppm);

    /// Get current sync state (thread-safe snapshot).
    ClockSyncState GetState() const;

    /// Convert driver time to device time (µs).
    uint64_t DriverToDeviceTime(uint64_t driver_us) const;

    /// Convert device time to driver time (µs).
    uint64_t DeviceToDriverTime(uint64_t device_us) const;

    /// Reset all state.
    void Reset();

private:
    Config config_;

    // Welford online variance for outlier detection
    int count_ = 0;
    double mean_ = 0.0;
    double m2_ = 0.0;

    // Min-RTT filter (NTP clock filter algorithm)
    // Selects the exchange with lowest RTT to reduce WiFi asymmetric delay bias.
    struct RttSample {
        int64_t raw_offset;
        int64_t raw_delay;
        int64_t rtt;
    };
    std::deque<RttSample> rtt_window_;

    // Median filter window
    std::deque<int64_t> median_buffer_;

    // Windowed PI output jitter for lock detection
    std::deque<int64_t> lock_window_;

    // PI controller state
    double integral_ = 0.0;
    int64_t offset_us_ = 0;
    int64_t delay_us_ = 0;
    int32_t rate_ratio_ppm_ = 0;
    int exchanges_ = 0;
    bool locked_ = false;
    int phase_count_ = 0;  // 0=store, 1=step, 2+=PI (linuxptp-style)

    // Offset-threshold lock (linuxptp-style: N consecutive below threshold)
    int stable_count_ = 10;  // Counts down from num_offset_values to 0

    // Diagnostic counters
    int outliers_rejected_ = 0;
    int lock_count_ = 0;
    int unlock_count_ = 0;
};

// ============================================================================
// SimTimeClock
// ============================================================================

/// Maps driver wall-clock to predicted simulation time using clock servo
/// offset and a simulation time anchor from state packets.
class SimTimeClock {
public:
    /// Update the anchor from a received state packet.
    void UpdateAnchor(const SimTimeAnchor& anchor);

    /// Predict current simulation time given driver wall-clock and servo.
    std::optional<double> PredictSimTime(uint64_t driver_now_us,
                                          const PTPClockServo& servo) const;

    /// Check if an anchor has been set.
    bool HasAnchor() const { return has_anchor_; }

    /// Reset anchor state.
    void Reset();

private:
    bool has_anchor_ = false;
    SimTimeAnchor anchor_;
};

// ============================================================================
// SyncClient
// ============================================================================

/// Dedicated UDP thread for peer-delay exchanges with the device.
/// Owns its own socket — independent of Driver's main socket.
class SyncClient {
public:
    struct Config {
        std::string host = "127.0.0.1";
        uint16_t port = 10001;           // Sync port (derived: control_port + 1000)
        uint32_t interval_ms = 100;     // Pdelay interval (10 Hz)
        uint32_t timeout_ms = 50;       // Recv timeout
        uint32_t max_consecutive_timeouts = 30;  // Reset servo after this many (~3s at 10Hz)
    };

    /// Construct a sync client.
    /// @param servo Reference to shared servo (protected by servo_mutex)
    /// @param servo_mutex Mutex protecting servo access
    /// @param config Sync client configuration
    SyncClient(PTPClockServo& servo, std::mutex& servo_mutex,
               const Config& config);

    ~SyncClient();

    // Non-copyable
    SyncClient(const SyncClient&) = delete;
    SyncClient& operator=(const SyncClient&) = delete;

    /// Start the sync thread.
    bool Start();

    /// Stop the sync thread.
    void Stop();

    /// Check if running.
    bool IsRunning() const { return running_.load(std::memory_order_acquire); }

private:
    void sync_thread_func();

    Config config_;
    PTPClockServo& servo_;
    std::mutex& servo_mutex_;

    int socket_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
    uint32_t seq_ = 0;
};

}  // namespace imujoco::driver

#endif  // IMUJOCO_DRIVER_TIME_SYNC_H
