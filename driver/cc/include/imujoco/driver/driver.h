// driver.h
// MuJoCo Driver - UDP client for communicating with iMuJoCo simulation
//
// Architecture:
//   - Dedicated RX thread for receiving state updates
//   - TX directly from caller's thread (UDP is full-duplex)
//   - Thread-safe public API
//   - Subscriber model for state updates (like ROS2)
//   - FlatBuffers for serialization
//   - UDP fragmentation for large state packets

#ifndef IMUJOCO_DRIVER_DRIVER_H
#define IMUJOCO_DRIVER_DRIVER_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

// FlatBuffers generated types
#include "control_generated.h"
#include "state_generated.h"

namespace imujoco::driver {

// Type aliases for FlatBuffers Object API types
// These provide a clean API while using FlatBuffers types directly
using ControlCommand = schema::ControlPacketT;
using SimulationState = schema::StatePacketT;

// Forward declarations
class UdpSocket;
class FragmentedSender;
class ReassemblyManager;

// Driver configuration
struct DriverConfig {
    // Target simulation host (default: localhost)
    std::string host = "127.0.0.1";

    // Target simulation UDP port (default: 9000)
    uint16_t port = 9000;

    // Local bind port (0 = ephemeral port)
    uint16_t local_port = 0;

    // Receive timeout in milliseconds (0 = no timeout)
    uint32_t timeout_ms = 100;

    // Auto-start receiving on connect (default: true)
    bool auto_start_receiving = true;
};

// Driver statistics
struct DriverStats {
    // Packets sent/received
    uint64_t packets_sent = 0;
    uint64_t packets_received = 0;
    uint64_t fragments_sent = 0;
    uint64_t fragments_received = 0;

    // Errors
    uint64_t send_errors = 0;
    uint64_t receive_errors = 0;

    // Timing
    double last_state_time = 0.0;
};

// Subscription ID for managing callbacks
using SubscriptionId = uint64_t;

// Callback types
using StateCallback = std::function<void(const SimulationState&)>;
using RawStateCallback = std::function<void(std::span<const uint8_t>)>;
using ErrorCallback = std::function<void(std::error_code ec, const std::string& message)>;

/// MuJoCo Driver - Thread-safe UDP client for iMuJoCo simulation
///
/// ## Threading Model
///
/// - RX: Dedicated thread for receiving state updates
/// - TX: Direct send from caller's thread (UDP is full-duplex)
/// - Public methods are thread-safe
///
/// ## Subscriber Model
///
/// Register callbacks to receive state updates:
/// ```cpp
/// auto id = driver.Subscribe([](const SimulationState& state) {
///     // Process state - runs on RX thread
/// });
/// driver.Unsubscribe(id);
/// ```
///
/// **Important:** Callbacks execute on the RX thread.
/// Keep callbacks lightweight or dispatch to your own thread.
///
/// ## Usage Example
///
/// ```cpp
/// Driver driver(config);
/// driver.Connect();
///
/// // Subscribe to state updates (runs on RX thread)
/// driver.Subscribe([](const SimulationState& state) {
///     std::cout << "Time: " << state.time << "\n";
/// });
///
/// // Send controls (runs on caller's thread)
/// driver.SendControl(ctrl);
/// ```
class Driver {
public:
    /// Create a driver with the given configuration.
    explicit Driver(const DriverConfig& config = {});

    /// Destructor - stops RX thread and closes socket.
    ~Driver();

    // Non-copyable, non-movable
    Driver(const Driver&) = delete;
    Driver& operator=(const Driver&) = delete;
    Driver(Driver&&) = delete;
    Driver& operator=(Driver&&) = delete;

    // =========================================================================
    // Connection Management (thread-safe)
    // =========================================================================

    /// Connect to the simulation.
    /// @return true if connected successfully
    bool Connect();

    /// Disconnect from the simulation.
    /// Stops receiving and closes the socket.
    void Disconnect();

    /// Check if connected.
    bool IsConnected() const;

    // =========================================================================
    // Control Operations (thread-safe)
    // =========================================================================

    /// Send a control command (non-blocking).
    /// Sends directly from caller's thread.
    /// @param cmd Control command to send
    void SendControl(const ControlCommand& cmd);

    /// Send control values (non-blocking).
    /// @param ctrl Control values for actuators
    void SendControl(std::span<const double> ctrl);

    // =========================================================================
    // FlatBuffers Raw API (thread-safe)
    // =========================================================================
    // These methods work with raw FlatBuffers buffers directly.
    // Useful for Python bindings and zero-copy scenarios.

    /// Send a pre-built FlatBuffers ControlPacket buffer (non-blocking).
    /// The buffer must be a valid FlatBuffers ControlPacket.
    /// @param data Raw FlatBuffers buffer
    void SendRaw(std::span<const uint8_t> data);

    // =========================================================================
    // Subscriber Model (thread-safe)
    // =========================================================================

    /// Subscribe to state updates (parsed).
    /// Callback is invoked on the RX thread for each received state.
    /// @param callback Function to call when state is received
    /// @return Subscription ID for unsubscribing
    SubscriptionId Subscribe(StateCallback callback);

    /// Subscribe to raw state updates (FlatBuffers buffer).
    /// Callback receives raw FlatBuffers StatePacket buffer.
    /// Useful for Python bindings and zero-copy scenarios.
    /// @param callback Function to call with raw buffer
    /// @return Subscription ID for unsubscribing
    SubscriptionId SubscribeRaw(RawStateCallback callback);

    /// Unsubscribe from state updates.
    /// @param id Subscription ID returned by Subscribe() or SubscribeRaw()
    /// @return true if subscription was found and removed
    bool Unsubscribe(SubscriptionId id);

    /// Set error callback.
    /// @param callback Function to call on error (runs on RX thread)
    void OnError(ErrorCallback callback);

    // =========================================================================
    // Receive Control (thread-safe)
    // =========================================================================

    /// Start the RX thread.
    /// Called automatically if config.auto_start_receiving is true.
    void StartReceiving();

    /// Stop the RX thread.
    void StopReceiving();

    /// Check if RX thread is running.
    bool IsReceiving() const;

    // =========================================================================
    // Statistics and Configuration (thread-safe)
    // =========================================================================

    /// Get current statistics.
    DriverStats GetStats() const;

    /// Reset statistics to zero.
    void ResetStats();

    /// Get the configuration.
    const DriverConfig& Config() const;

private:
    // RX thread function
    void rx_thread_func();

    // Send fragments (called from TX path)
    bool send_fragments(const uint8_t* data, size_t size);

    // Parse FlatBuffers message
    std::optional<SimulationState> parse_state_packet(const uint8_t* data, size_t size);

    // Dispatch state to subscribers (both parsed and raw)
    void dispatch_state(const uint8_t* data, size_t size);

    // Report error to callback
    void report_error(std::error_code ec, const std::string& message);

    // Configuration
    DriverConfig config_;

    // UDP socket (thread-safe for concurrent send/receive)
    std::unique_ptr<UdpSocket> socket_;

    // TX: Fragmentation (protected by tx_mutex_)
    std::mutex tx_mutex_;
    std::unique_ptr<FragmentedSender> sender_;
    std::atomic<uint32_t> sequence_{0};

    // RX: Thread and reassembly (protected by rx_mutex_)
    mutable std::mutex rx_mutex_;
    std::thread rx_thread_;
    std::unique_ptr<ReassemblyManager> reassembler_;
    std::vector<uint8_t> recv_buffer_;

    // Connection state (protected by connect_mutex_)
    mutable std::mutex connect_mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> receiving_{false};

    // Subscribers (protected by mutex)
    mutable std::mutex subscribers_mutex_;
    std::map<SubscriptionId, StateCallback> subscribers_;
    std::map<SubscriptionId, RawStateCallback> raw_subscribers_;
    std::atomic<SubscriptionId> next_subscription_id_{1};

    // Error callback
    mutable std::mutex error_callback_mutex_;
    ErrorCallback error_callback_;

    // Statistics
    mutable std::mutex stats_mutex_;
    DriverStats stats_;
};

}  // namespace imujoco::driver

#endif  // IMUJOCO_DRIVER_DRIVER_H
