// driver.cc
// MuJoCo Driver implementation
// - RX: Dedicated thread for receiving
// - TX: Direct send from caller's thread

#include "imujoco/driver/driver.h"

#include "control_generated.h"
#include "fragment.h"
#include "state_generated.h"
#include "udp_socket.h"

namespace imujoco::driver {

// ============================================================================
// Construction / Destruction
// ============================================================================

Driver::Driver(const DriverConfig& config)
    : config_(config),
      socket_(std::make_unique<UdpSocket>()),
      sender_(std::make_unique<FragmentedSender>()),
      reassembler_(std::make_unique<ReassemblyManager>()) {
    recv_buffer_.resize(kMaxUDPPayload);
}

Driver::~Driver() {
    Disconnect();
}

// ============================================================================
// Connection Management
// ============================================================================

bool Driver::Connect() {
    if (connected_.load(std::memory_order_acquire)) {
        return true;  // Already connected
    }

    if (!socket_->Initialize(config_.local_port)) {
        return false;
    }

    if (!socket_->SetRemote(config_.host, config_.port)) {
        socket_->Close();
        return false;
    }

    connected_.store(true, std::memory_order_release);

    if (config_.auto_start_receiving) {
        StartReceiving();
    }

    return true;
}

void Driver::Disconnect() {
    // Stop RX thread first
    StopReceiving();

    // Close socket
    connected_.store(false, std::memory_order_release);
    socket_->Close();
}

bool Driver::IsConnected() const {
    return connected_.load(std::memory_order_acquire);
}

// ============================================================================
// Control Operations (TX)
// ============================================================================

void Driver::SendControl(const ControlCommand& cmd) {
    if (!IsConnected()) {
        return;
    }

    // Use a mutable copy if we need to set sequence
    ControlCommand packet = cmd;
    if (packet.sequence == 0) {
        packet.sequence = ++sequence_;
    }

    // Use FlatBuffers Pack to convert from Object API type (ControlPacketT)
    // IMPORTANT: Must use FinishControlPacketBuffer to include file identifier "CTPK"
    flatbuffers::FlatBufferBuilder builder(256);
    auto offset = imujoco::schema::ControlPacket::Pack(builder, &packet);
    imujoco::schema::FinishControlPacketBuffer(builder, offset);

    // Send with fragmentation
    send_fragments(builder.GetBufferPointer(), builder.GetSize());
}

void Driver::SendControl(std::span<const double> ctrl) {
    ControlCommand cmd;
    cmd.ctrl.assign(ctrl.begin(), ctrl.end());
    SendControl(cmd);
}

// ============================================================================
// FlatBuffers Raw API
// ============================================================================

void Driver::SendRaw(std::span<const uint8_t> data) {
    if (!IsConnected()) {
        return;
    }
    send_fragments(data.data(), data.size());
}

bool Driver::send_fragments(const uint8_t* data, size_t size) {
    std::lock_guard<std::mutex> lock(tx_mutex_);

    auto fragments = sender_->FragmentMessage(data, size);
    if (fragments.empty()) {
        std::lock_guard<std::mutex> slock(stats_mutex_);
        stats_.send_errors++;
        return false;
    }

    for (const auto& fragment : fragments) {
        auto sent = socket_->Send(std::span<const uint8_t>(fragment));
        if (sent < 0) {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.send_errors++;
            return false;
        }
        {
            std::lock_guard<std::mutex> slock(stats_mutex_);
            stats_.fragments_sent++;
        }
    }

    {
        std::lock_guard<std::mutex> slock(stats_mutex_);
        stats_.packets_sent++;
    }
    return true;
}

// ============================================================================
// Subscriber Model
// ============================================================================

SubscriptionId Driver::Subscribe(StateCallback callback) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    SubscriptionId id = next_subscription_id_++;
    subscribers_[id] = std::move(callback);
    return id;
}

SubscriptionId Driver::SubscribeRaw(RawStateCallback callback) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    SubscriptionId id = next_subscription_id_++;
    raw_subscribers_[id] = std::move(callback);
    return id;
}

bool Driver::Unsubscribe(SubscriptionId id) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    if (subscribers_.erase(id) > 0) {
        return true;
    }
    return raw_subscribers_.erase(id) > 0;
}

void Driver::OnError(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(error_callback_mutex_);
    error_callback_ = std::move(callback);
}

// ============================================================================
// RX Thread Control
// ============================================================================

void Driver::StartReceiving() {
    bool expected = false;
    if (receiving_.compare_exchange_strong(expected, true)) {
        rx_thread_ = std::thread([this]() { rx_thread_func(); });
    }
}

void Driver::StopReceiving() {
    receiving_.store(false, std::memory_order_release);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

bool Driver::IsReceiving() const {
    return receiving_.load(std::memory_order_acquire);
}

// ============================================================================
// RX Thread Function
// ============================================================================

void Driver::rx_thread_func() {
    while (receiving_.load(std::memory_order_acquire) && IsConnected()) {
        auto received = socket_->Receive(recv_buffer_, config_.timeout_ms);

        if (received == 0) {
            // Timeout - cleanup stale reassembly slots
            reassembler_->CleanupStale();
            continue;
        }

        if (received < 0) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.receive_errors++;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.fragments_received++;
        }

        // Try to reassemble
        auto result = reassembler_->ProcessFragment(
            recv_buffer_.data(), static_cast<size_t>(received));

        if (!result.complete) {
            continue;
        }

        // Validate the packet before dispatching
        auto verifier = flatbuffers::Verifier(result.data, result.size);
        if (!imujoco::schema::VerifyStatePacketBuffer(verifier)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.receive_errors++;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.packets_received++;
            auto packet = imujoco::schema::GetStatePacket(result.data);
            stats_.last_state_time = packet->time();
        }

        dispatch_state(result.data, result.size);
    }
}

// ============================================================================
// Helpers
// ============================================================================

std::optional<SimulationState> Driver::parse_state_packet(const uint8_t* data, size_t size) {
    auto verifier = flatbuffers::Verifier(data, size);
    if (!imujoco::schema::VerifyStatePacketBuffer(verifier)) {
        return std::nullopt;
    }

    // Use FlatBuffers UnPack to convert to Object API type (StatePacketT)
    auto packet = imujoco::schema::GetStatePacket(data);
    SimulationState state;
    packet->UnPackTo(&state);
    return state;
}

void Driver::dispatch_state(const uint8_t* data, size_t size) {
    // Copy subscriber maps to avoid holding mutex during callbacks
    // This prevents deadlock if callbacks try to Subscribe/Unsubscribe
    std::map<SubscriptionId, RawStateCallback> raw_subs_copy;
    std::map<SubscriptionId, StateCallback> subs_copy;
    {
        std::lock_guard<std::mutex> lock(subscribers_mutex_);
        raw_subs_copy = raw_subscribers_;
        subs_copy = subscribers_;
    }

    // Dispatch to raw subscribers first (no parsing needed)
    for (const auto& [id, callback] : raw_subs_copy) {
        try {
            callback(std::span<const uint8_t>(data, size));
        } catch (const std::exception& e) {
            report_error(std::error_code(), std::string("Raw subscriber exception: ") + e.what());
        }
    }

    // Parse and dispatch to regular subscribers only if there are any
    if (!subs_copy.empty()) {
        auto state = parse_state_packet(data, size);
        if (state) {
            for (const auto& [id, callback] : subs_copy) {
                try {
                    callback(*state);
                } catch (const std::exception& e) {
                    report_error(std::error_code(), std::string("Subscriber exception: ") + e.what());
                }
            }
        }
    }
}

void Driver::report_error(std::error_code ec, const std::string& message) {
    std::lock_guard<std::mutex> lock(error_callback_mutex_);
    if (error_callback_) {
        try {
            error_callback_(ec, message);
        } catch (...) {
            // Ignore
        }
    }
}

// ============================================================================
// Statistics
// ============================================================================

DriverStats Driver::GetStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void Driver::ResetStats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = DriverStats{};
}

const DriverConfig& Driver::Config() const {
    return config_;
}

}  // namespace imujoco::driver
