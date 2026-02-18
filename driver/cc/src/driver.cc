// driver.cc
// MuJoCo Driver implementation
// - RX: Dedicated thread for receiving
// - TX: Direct send from caller's thread

#include "imujoco/driver/driver.h"

#include "control_generated.h"
#include "fragment.h"
#include "state_generated.h"
#include "udp_socket.h"

#include <chrono>

namespace imujoco::driver {

// ============================================================================
// Construction / Destruction
// ============================================================================

Driver::Driver(const DriverConfig& config)
    : config_(config),
      socket_(std::make_unique<UdpSocket>()),
      sender_(std::make_unique<FragmentedSender>()),
      tx_builder_(std::make_unique<flatbuffers::FlatBufferBuilder>(256)),
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
    std::lock_guard<std::mutex> lock(connect_mutex_);

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
    // Use unique_lock to allow releasing before StopReceiving() to avoid deadlock
    // (StopReceiving acquires rx_mutex_, and we must not hold connect_mutex_ while
    // acquiring rx_mutex_ to prevent lock ordering issues)
    {
        std::lock_guard<std::mutex> lock(connect_mutex_);

        if (!connected_.load(std::memory_order_acquire)) {
            return;  // Already disconnected
        }

        connected_.store(false, std::memory_order_release);
    }

    // Stop RX thread after marking as disconnected (without holding connect_mutex_)
    StopReceiving();

    // Close socket once RX thread has stopped
    {
        std::lock_guard<std::mutex> lock(connect_mutex_);
        socket_->Close();
    }
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

    // Use a mutable copy if we need to set sequence or timestamp
    ControlCommand packet = cmd;
    if (packet.sequence == 0) {
        packet.sequence = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    }
    if (packet.host_timestamp_us == 0) {
        auto now = std::chrono::steady_clock::now();
        packet.host_timestamp_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()).count());
    }

    // Build and send under tx_mutex_ to reuse the pooled builder
    std::lock_guard<std::mutex> lock(tx_mutex_);
    tx_builder_->Clear();
    auto offset = imujoco::schema::ControlPacket::Pack(*tx_builder_, &packet);
    imujoco::schema::FinishControlPacketBuffer(*tx_builder_, offset);

    // Send with fragmentation (already holds tx_mutex_)
    auto fragments = sender_->FragmentMessage(
        tx_builder_->GetBufferPointer(), tx_builder_->GetSize());
    if (fragments.empty()) {
        stat_send_errors_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    for (const auto& fragment : fragments) {
        auto sent = socket_->Send(std::span<const uint8_t>(fragment));
        if (sent < 0) {
            stat_send_errors_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        stat_fragments_sent_.fetch_add(1, std::memory_order_relaxed);
    }
    stat_packets_sent_.fetch_add(1, std::memory_order_relaxed);
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
        stat_send_errors_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    for (const auto& fragment : fragments) {
        auto sent = socket_->Send(std::span<const uint8_t>(fragment));
        if (sent < 0) {
            stat_send_errors_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        stat_fragments_sent_.fetch_add(1, std::memory_order_relaxed);
    }

    stat_packets_sent_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// ============================================================================
// Subscriber Model
// ============================================================================

SubscriptionId Driver::Subscribe(StateCallback callback) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    SubscriptionId id = next_subscription_id_++;
    auto copy = std::make_shared<std::map<SubscriptionId, StateCallback>>(*subscribers_);
    (*copy)[id] = std::move(callback);
    subscribers_ = std::move(copy);
    return id;
}

SubscriptionId Driver::SubscribeRaw(RawStateCallback callback) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    SubscriptionId id = next_subscription_id_++;
    auto copy = std::make_shared<std::map<SubscriptionId, RawStateCallback>>(*raw_subscribers_);
    (*copy)[id] = std::move(callback);
    raw_subscribers_ = std::move(copy);
    return id;
}

bool Driver::Unsubscribe(SubscriptionId id) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    {
        auto copy = std::make_shared<std::map<SubscriptionId, StateCallback>>(*subscribers_);
        if (copy->erase(id) > 0) {
            subscribers_ = std::move(copy);
            return true;
        }
    }
    auto copy = std::make_shared<std::map<SubscriptionId, RawStateCallback>>(*raw_subscribers_);
    if (copy->erase(id) > 0) {
        raw_subscribers_ = std::move(copy);
        return true;
    }
    return false;
}

void Driver::OnError(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(error_callback_mutex_);
    error_callback_ = std::move(callback);
}

// ============================================================================
// RX Thread Control
// ============================================================================

void Driver::StartReceiving() {
    std::lock_guard<std::mutex> lock(rx_mutex_);

    if (receiving_.load(std::memory_order_acquire)) {
        return;  // Already receiving
    }

    receiving_.store(true, std::memory_order_release);
    rx_thread_ = std::thread([this]() { rx_thread_func(); });
}

void Driver::StopReceiving() {
    std::lock_guard<std::mutex> lock(rx_mutex_);

    if (!receiving_.load(std::memory_order_acquire)) {
        return;  // Not receiving
    }

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
            stat_receive_errors_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        stat_fragments_received_.fetch_add(1, std::memory_order_relaxed);

        // Try to reassemble
        auto result = reassembler_->ProcessFragment(
            recv_buffer_.data(), static_cast<size_t>(received));

        if (!result.complete) {
            continue;
        }

        // Validate the packet before dispatching
        auto verifier = flatbuffers::Verifier(result.data, result.size);
        if (!imujoco::schema::VerifyStatePacketBuffer(verifier)) {
            stat_receive_errors_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        stat_packets_received_.fetch_add(1, std::memory_order_relaxed);
        auto packet = imujoco::schema::GetStatePacket(result.data);
        stat_last_state_time_.store(packet->time(), std::memory_order_relaxed);

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
    // Grab shared_ptr snapshots (one atomic ref-count increment each, no map copy)
    std::shared_ptr<const std::map<SubscriptionId, RawStateCallback>> raw_subs;
    std::shared_ptr<const std::map<SubscriptionId, StateCallback>> subs;
    {
        std::lock_guard<std::mutex> lock(subscribers_mutex_);
        raw_subs = raw_subscribers_;
        subs = subscribers_;
    }

    // Dispatch to raw subscribers first (no parsing needed)
    for (const auto& [id, callback] : *raw_subs) {
        try {
            callback(std::span<const uint8_t>(data, size));
        } catch (const std::exception& e) {
            report_error(std::error_code(), std::string("Raw subscriber exception: ") + e.what());
        } catch (...) {
            report_error(std::error_code(), "Raw subscriber threw unknown exception");
        }
    }

    // Parse and dispatch to regular subscribers only if there are any
    if (!subs->empty()) {
        auto state = parse_state_packet(data, size);
        if (state) {
            for (const auto& [id, callback] : *subs) {
                try {
                    callback(*state);
                } catch (const std::exception& e) {
                    report_error(std::error_code(), std::string("Subscriber exception: ") + e.what());
                } catch (...) {
                    report_error(std::error_code(), "Subscriber threw unknown exception");
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
    DriverStats s;
    s.packets_sent = stat_packets_sent_.load(std::memory_order_relaxed);
    s.packets_received = stat_packets_received_.load(std::memory_order_relaxed);
    s.fragments_sent = stat_fragments_sent_.load(std::memory_order_relaxed);
    s.fragments_received = stat_fragments_received_.load(std::memory_order_relaxed);
    s.send_errors = stat_send_errors_.load(std::memory_order_relaxed);
    s.receive_errors = stat_receive_errors_.load(std::memory_order_relaxed);
    s.last_state_time = stat_last_state_time_.load(std::memory_order_relaxed);
    return s;
}

void Driver::ResetStats() {
    stat_packets_sent_.store(0, std::memory_order_relaxed);
    stat_packets_received_.store(0, std::memory_order_relaxed);
    stat_fragments_sent_.store(0, std::memory_order_relaxed);
    stat_fragments_received_.store(0, std::memory_order_relaxed);
    stat_send_errors_.store(0, std::memory_order_relaxed);
    stat_receive_errors_.store(0, std::memory_order_relaxed);
    stat_last_state_time_.store(0.0, std::memory_order_relaxed);
}

const DriverConfig& Driver::Config() const {
    return config_;
}

}  // namespace imujoco::driver
