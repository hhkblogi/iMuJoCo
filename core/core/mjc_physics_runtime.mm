// MJPhysicsRuntime.mm
// C++ implementation of MuJoCo physics simulation runtime
// Uses lock-free ring buffer with C++20 atomic wait/notify for lowest latency

#include "mjc_physics_runtime.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

// Network includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <errno.h>

// MARK: - Constants

namespace {

constexpr int kRingBufferSize = 3;      // Triple buffering
constexpr int kMaxGeoms = 10000;        // Maximum geoms per frame
constexpr int kErrorLength = 1024;      // Error buffer size

// Physics timing constants
constexpr double kSyncMisalign = 0.1;
constexpr double kSimRefreshFraction = 0.7;

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

}  // namespace

// MARK: - GeomInstance (Data for instanced rendering)

struct GeomInstance {
    // Transform
    float pos[3];           // World position
    float mat[9];           // 3x3 rotation matrix
    float size[3];          // Geom size parameters

    // Visual properties
    float rgba[4];          // Color
    int32_t type;           // Geom type (sphere, capsule, box, etc.)
    float emission;         // Emission
    float specular;         // Specular
    float shininess;        // Shininess
};

// MARK: - FrameData (Complete frame for rendering)
// Note: This struct must have the same memory layout as MJFrameData in the header!

struct FrameData {
    // Geom instances for rendering (matches MJFrameData.geoms)
    GeomInstance geoms[kMaxGeoms];
    int32_t geom_count;

    // Camera state (matches MJFrameData camera fields)
    float camera_pos[3];       // Camera world position (computed)
    float camera_lookat[3];    // Look-at target
    float camera_azimuth;
    float camera_elevation;
    float camera_distance;

    // Timing
    double simulation_time;
    int32_t steps_per_second;

    // Frame sequence number (for debugging)
    uint64_t frame_number;

    FrameData() : geom_count(0), simulation_time(0.0), steps_per_second(0), frame_number(0) {
        camera_pos[0] = camera_pos[1] = camera_pos[2] = 0;
        camera_lookat[0] = camera_lookat[1] = camera_lookat[2] = 0;
        camera_azimuth = 0;
        camera_elevation = 0;
        camera_distance = 3;
    }
};

// MARK: - LockFreeRingBuffer

class LockFreeRingBuffer {
public:
    LockFreeRingBuffer() : write_index_(0), read_index_(0), frame_count_(0) {
        // Buffers are initialized by FrameData constructor
    }

    // Physics thread: get buffer to write to
    FrameData* BeginWrite() {
        return &buffers_[write_index_.load(std::memory_order_relaxed)];
    }

    // Physics thread: publish written frame
    void EndWrite() {
        int idx = write_index_.load(std::memory_order_relaxed);

        // Update read index to point to this buffer (latest complete frame)
        read_index_.store(idx, std::memory_order_release);

        // Move write index to next buffer
        write_index_.store((idx + 1) % kRingBufferSize, std::memory_order_relaxed);

        // Increment frame count and notify waiters
        frame_count_.fetch_add(1, std::memory_order_release);
        frame_count_.notify_one();
    }

    // Render thread: wait for new frame (blocking)
    const FrameData* WaitForFrame(uint64_t last_frame) {
        // Wait until frame_count changes
        frame_count_.wait(last_frame, std::memory_order_acquire);
        return &buffers_[read_index_.load(std::memory_order_acquire)];
    }

    // Render thread: get latest frame (non-blocking)
    const FrameData* GetLatestFrame() const {
        return &buffers_[read_index_.load(std::memory_order_acquire)];
    }

    // Get current frame count (for wait comparison)
    uint64_t GetFrameCount() const {
        return frame_count_.load(std::memory_order_acquire);
    }

    // Signal exit to unblock waiters
    void SignalExit() {
        frame_count_.fetch_add(1, std::memory_order_release);
        frame_count_.notify_all();
    }

private:
    std::array<FrameData, kRingBufferSize> buffers_;

    alignas(64) std::atomic<int> write_index_;    // Cache line aligned
    alignas(64) std::atomic<int> read_index_;     // Cache line aligned
    alignas(64) std::atomic<uint64_t> frame_count_;  // For wait/notify
};

// MARK: - UDP Packet Structures

#pragma pack(push, 1)

struct ControlPacket {
    uint32_t magic;       // MJ_PACKET_MAGIC_CTRL
    uint32_t sequence;
    int32_t nu;           // Number of control values
    uint32_t reserved;
    // Followed by: double ctrl[nu]
};

struct StatePacketHeader {
    uint32_t magic;       // MJ_PACKET_MAGIC_STATE
    uint32_t sequence;
    double time;
    int32_t nq;
    int32_t nv;
    int32_t nu;
    int32_t nsensordata;
    double energy[2];     // [potential, kinetic]
    // Followed by: double qpos[nq], qvel[nv], ctrl[nu], sensordata[nsensordata]
};

#pragma pack(pop)

// MARK: - UDPServer

class UDPServer {
public:
    UDPServer() : socket_fd_(-1), port_(0), has_client_(false),
                  packets_received_(0), packets_sent_(0), send_sequence_(0) {}

    ~UDPServer() {
        Close();
    }

    bool Start(uint16_t port) {
        if (port == 0) return false;

        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            printf("[UDPServer] Failed to create socket\n");
            return false;
        }

        // Set non-blocking
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        // Allow address reuse
        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Bind to port
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            printf("[UDPServer] Failed to bind to port %d\n", port);
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        port_ = port;
        printf("[UDPServer] Listening on port %d\n", port);
        return true;
    }

    void Close() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
            printf("[UDPServer] Closed port %d\n", port_);
        }
        port_ = 0;
        has_client_ = false;
    }

    bool IsActive() const { return socket_fd_ >= 0; }
    uint16_t GetPort() const { return port_; }
    bool HasClient() const { return has_client_; }
    uint32_t GetPacketsReceived() const { return packets_received_; }
    uint32_t GetPacketsSent() const { return packets_sent_; }

    // Receive control packet (non-blocking)
    // Returns: number of control values received (>=0), or -1 if no packet available
    int ReceiveControl(double* ctrl_out, int max_ctrl) {
        if (socket_fd_ < 0) return -1;

        uint8_t buffer[65535];
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                    (struct sockaddr*)&client_addr, &addr_len);

        if (recv_len <= 0) {
            return -1;  // No packet available
        }

        printf("[UDPServer] Received %zd bytes from %s:%d\n",
               recv_len, inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        // Validate minimum header size
        if (recv_len < (ssize_t)sizeof(ControlPacket)) {
            printf("[UDPServer] Packet too small for header\n");
            return -1;
        }

        const ControlPacket* packet = reinterpret_cast<const ControlPacket*>(buffer);

        // Validate magic number
        if (packet->magic != MJ_PACKET_MAGIC_CTRL) {
            printf("[UDPServer] Invalid magic: 0x%08X (expected 0x%08X)\n",
                   packet->magic, MJ_PACKET_MAGIC_CTRL);
            return -1;
        }

        // Store client address for sending state back
        client_addr_ = client_addr;
        has_client_ = true;
        packets_received_++;

        // Extract control values
        int nu = packet->nu;
        printf("[UDPServer] Control packet: seq=%u, nu=%d\n", packet->sequence, nu);

        // Packet with no controls is valid (just a "tick" request)
        if (nu <= 0) return 0;

        // Validate packet size
        size_t expected_size = sizeof(ControlPacket) + nu * sizeof(double);
        if ((size_t)recv_len < expected_size) {
            printf("[UDPServer] Packet too small for %d controls\n", nu);
            return -1;
        }

        // Copy control values
        const double* ctrl_data = reinterpret_cast<const double*>(buffer + sizeof(ControlPacket));
        int copy_count = std::min(nu, max_ctrl);
        if (copy_count > 0 && ctrl_out != nullptr) {
            memcpy(ctrl_out, ctrl_data, copy_count * sizeof(double));
        }

        return copy_count;
    }

    // Send state packet to last known client
    bool SendState(const mjModel* model, const mjData* data) {
        if (socket_fd_ < 0 || !has_client_ || !model || !data) {
            if (socket_fd_ < 0) printf("[UDPServer] SendState: socket not open\n");
            else if (!has_client_) printf("[UDPServer] SendState: no client\n");
            return false;
        }

        // Calculate packet size
        int nq = model->nq;
        int nv = model->nv;
        int nu = model->nu;
        int nsensordata = model->nsensordata;

        size_t data_size = (nq + nv + nu + nsensordata) * sizeof(double);
        size_t packet_size = sizeof(StatePacketHeader) + data_size;

        // Allocate buffer (could use static buffer for better performance)
        std::vector<uint8_t> buffer(packet_size);

        // Fill header
        StatePacketHeader* header = reinterpret_cast<StatePacketHeader*>(buffer.data());
        header->magic = MJ_PACKET_MAGIC_STATE;
        header->sequence = send_sequence_++;
        header->time = data->time;
        header->nq = nq;
        header->nv = nv;
        header->nu = nu;
        header->nsensordata = nsensordata;
        header->energy[0] = data->energy[0];  // Potential
        header->energy[1] = data->energy[1];  // Kinetic

        // Copy data arrays
        double* out = reinterpret_cast<double*>(buffer.data() + sizeof(StatePacketHeader));

        if (nq > 0) {
            memcpy(out, data->qpos, nq * sizeof(double));
            out += nq;
        }
        if (nv > 0) {
            memcpy(out, data->qvel, nv * sizeof(double));
            out += nv;
        }
        if (nu > 0) {
            memcpy(out, data->ctrl, nu * sizeof(double));
            out += nu;
        }
        if (nsensordata > 0) {
            memcpy(out, data->sensordata, nsensordata * sizeof(double));
        }

        // Send packet
        ssize_t sent = sendto(socket_fd_, buffer.data(), packet_size, 0,
                              (struct sockaddr*)&client_addr_, sizeof(client_addr_));

        if (sent == (ssize_t)packet_size) {
            packets_sent_++;
            if (packets_sent_ <= 5 || packets_sent_ % 100 == 0) {  // Log first 5 and every 100
                printf("[UDPServer] Sent state packet #%u (%zu bytes) to %s:%d\n",
                       packets_sent_, packet_size,
                       inet_ntoa(client_addr_.sin_addr), ntohs(client_addr_.sin_port));
            }
            return true;
        } else {
            printf("[UDPServer] sendto failed: sent %zd of %zu bytes, errno=%d\n",
                   sent, packet_size, errno);
        }
        return false;
    }

private:
    int socket_fd_;
    uint16_t port_;
    struct sockaddr_in client_addr_;
    bool has_client_;
    uint32_t packets_received_;
    uint32_t packets_sent_;
    uint32_t send_sequence_;
};

// MARK: - MJSimulationRuntime Class

class MJSimulationRuntime {
public:
    explicit MJSimulationRuntime(const MJRuntimeConfig& config)
        : instance_index_(config.instanceIndex)
        , target_fps_(config.targetFPS > 0 ? config.targetFPS : 60.0)
        , busy_wait_(config.busyWait)
        , udp_port_(config.udpPort > 0 ? config.udpPort : MJ_DEFAULT_UDP_PORT + config.instanceIndex)
        , realtime_factor_(1.0)
        , state_(MJRuntimeStateInactive)
        , model_(nullptr)
        , data_(nullptr)
        , exit_requested_(false)
        , speed_changed_(false)
        , last_read_frame_(0) {

        printf("[MJRuntime] Creating instance %d (lock-free ring buffer, UDP port %d)\n",
               config.instanceIndex, udp_port_);

        // Initialize visualization structures (still needed for mjv_updateScene)
        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&option_);

        memset(&scene_, 0, sizeof(mjvScene));
        mjv_makeScene(nullptr, &scene_, kMaxGeoms);

        // Set default camera position
        camera_.type = mjCAMERA_FREE;
        camera_.azimuth = 90;
        camera_.elevation = -15;
        camera_.distance = 3.0;
        camera_.lookat[0] = 0;
        camera_.lookat[1] = 0;
        camera_.lookat[2] = 0.5;

        printf("[MJRuntime] Instance %d ready\n", config.instanceIndex);
    }

    ~MJSimulationRuntime() {
        printf("[MJRuntime] Destroying instance %d\n", instance_index_);
        Stop();
        Unload();
        if (scene_.maxgeom > 0) {
            mjv_freeScene(&scene_);
        }
        printf("[MJRuntime] Instance %d destroyed\n", instance_index_);
    }

    // MARK: - Public Methods (PascalCase)

    bool LoadModel(const char* xml_path, char* error_buffer, int32_t error_buffer_size) {
        printf("[MJRuntime] LoadModel: %s\n", xml_path ? xml_path : "(null)");
        Stop();

        // No mutex needed - physics thread is stopped
        if (data_) {
            mj_deleteData(data_);
            data_ = nullptr;
        }
        if (model_) {
            mj_deleteModel(model_);
            model_ = nullptr;
        }

        char load_error[kErrorLength] = "";
        mjModel* mnew = mj_loadXML(xml_path, nullptr, load_error, kErrorLength);

        if (!mnew) {
            if (error_buffer && error_buffer_size > 0) {
                strncpy(error_buffer, load_error, error_buffer_size - 1);
                error_buffer[error_buffer_size - 1] = '\0';
            }
            state_ = MJRuntimeStateInactive;
            return false;
        }

        mjData* dnew = mj_makeData(mnew);
        if (!dnew) {
            mj_deleteModel(mnew);
            if (error_buffer && error_buffer_size > 0) {
                strncpy(error_buffer, "Failed to create mjData", error_buffer_size - 1);
            }
            state_ = MJRuntimeStateInactive;
            return false;
        }

        model_ = mnew;
        data_ = dnew;

        if (model_->nkey > 0) {
            mj_resetDataKeyframe(model_, data_, 0);
        }

        mj_forward(model_, data_);

        // Write initial frame to ring buffer
        write_frame_to_buffer();

        state_ = MJRuntimeStateLoaded;
        return true;
    }

    bool LoadModelXML(const char* xml_string, char* error_buffer, int32_t error_buffer_size) {
        Stop();

        if (data_) {
            mj_deleteData(data_);
            data_ = nullptr;
        }
        if (model_) {
            mj_deleteModel(model_);
            model_ = nullptr;
        }

        char load_error[kErrorLength] = "";
        mjSpec* spec = mj_parseXMLString(xml_string, nullptr, load_error, kErrorLength);
        if (!spec) {
            if (error_buffer && error_buffer_size > 0) {
                strncpy(error_buffer, load_error, error_buffer_size - 1);
                error_buffer[error_buffer_size - 1] = '\0';
            }
            state_ = MJRuntimeStateInactive;
            return false;
        }

        mjModel* mnew = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);

        if (!mnew) {
            if (error_buffer && error_buffer_size > 0) {
                strncpy(error_buffer, "Failed to compile model", error_buffer_size - 1);
            }
            state_ = MJRuntimeStateInactive;
            return false;
        }

        mjData* dnew = mj_makeData(mnew);
        if (!dnew) {
            mj_deleteModel(mnew);
            if (error_buffer && error_buffer_size > 0) {
                strncpy(error_buffer, "Failed to create mjData", error_buffer_size - 1);
            }
            state_ = MJRuntimeStateInactive;
            return false;
        }

        model_ = mnew;
        data_ = dnew;

        if (model_->nkey > 0) {
            mj_resetDataKeyframe(model_, data_, 0);
        }

        mj_forward(model_, data_);
        write_frame_to_buffer();

        state_ = MJRuntimeStateLoaded;
        return true;
    }

    void Unload() {
        Stop();

        if (data_) {
            mj_deleteData(data_);
            data_ = nullptr;
        }
        if (model_) {
            mj_deleteModel(model_);
            model_ = nullptr;
        }

        state_ = MJRuntimeStateInactive;
    }

    void Start() {
        if (!model_ || !data_) return;
        if (state_ == MJRuntimeStateRunning) return;

        // Start UDP server
        if (udp_port_ > 0 && !udp_server_.IsActive()) {
            udp_server_.Start(udp_port_);
        }

        state_ = MJRuntimeStateRunning;
        exit_requested_ = false;
        speed_changed_ = true;

        physics_thread_ = std::thread(&MJSimulationRuntime::physics_loop, this);
    }

    void Pause() {
        if (state_ != MJRuntimeStateRunning) return;
        state_ = MJRuntimeStatePaused;
        Stop();
    }

    void Stop() {
        exit_requested_ = true;
        ring_buffer_.SignalExit();  // Unblock any waiters

        if (physics_thread_.joinable()) {
            physics_thread_.join();
        }

        // Close UDP server
        udp_server_.Close();
    }

    void Reset() {
        // Safe to call while running - physics loop will see the reset
        if (model_ && data_) {
            if (model_->nkey > 0) {
                mj_resetDataKeyframe(model_, data_, 0);
            } else {
                mj_resetData(model_, data_);
            }
            mj_forward(model_, data_);
            speed_changed_ = true;
        }
    }

    void Step() {
        if (model_ && data_ && state_ != MJRuntimeStateRunning) {
            mj_step(model_, data_);
            write_frame_to_buffer();
        }
    }

    MJRuntimeState GetState() const { return state_; }

    MJRuntimeStats GetStats() const {
        const FrameData* frame = ring_buffer_.GetLatestFrame();
        MJRuntimeStats stats;
        stats.simulationTime = frame->simulation_time;
        stats.measuredSlowdown = 1.0;  // TODO: track this
        stats.timestep = model_ ? model_->opt.timestep : 0.002;
        stats.stepsPerSecond = frame->steps_per_second;
        // Network stats
        stats.udpPort = udp_server_.IsActive() ? udp_server_.GetPort() : 0;
        stats.packetsReceived = udp_server_.GetPacketsReceived();
        stats.packetsSent = udp_server_.GetPacketsSent();
        stats.hasClient = udp_server_.HasClient();
        return stats;
    }

    // MARK: - Ring Buffer Access (for rendering)

    // Wait for new frame (blocks until physics produces one)
    const FrameData* WaitForFrame() {
        uint64_t last = last_read_frame_;
        const FrameData* frame = ring_buffer_.WaitForFrame(last);
        last_read_frame_ = ring_buffer_.GetFrameCount();
        return frame;
    }

    // Get latest frame without waiting (may return same frame twice)
    const FrameData* GetLatestFrame() const {
        return ring_buffer_.GetLatestFrame();
    }

    uint64_t GetFrameCount() const {
        return ring_buffer_.GetFrameCount();
    }

    // MARK: - Legacy Scene Access (for compatibility)

    // These still work but use the ring buffer internally
    const mjvScene* GetScene() const { return &scene_; }
    mjvCamera* GetCamera() { return &camera_; }
    mjvOption* GetOption() { return &option_; }
    const mjModel* GetModel() const { return model_; }
    const mjData* GetData() const { return data_; }

    void UpdateScene() {
        if (model_ && data_) {
            mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
        }
    }

    // MARK: - Camera Control

    void SetCameraAzimuth(double azimuth) {
        camera_.azimuth = azimuth;
    }

    void SetCameraElevation(double elevation) {
        camera_.elevation = std::max(-89.0, std::min(89.0, elevation));
    }

    void SetCameraDistance(double distance) {
        camera_.distance = std::max(0.1, std::min(100.0, distance));
    }

    void SetCameraLookat(double x, double y, double z) {
        camera_.lookat[0] = x;
        camera_.lookat[1] = y;
        camera_.lookat[2] = z;
    }

    void ResetCamera() {
        camera_.azimuth = 90;
        camera_.elevation = -15;
        camera_.distance = 3.0;
        camera_.lookat[0] = 0;
        camera_.lookat[1] = 0;
        camera_.lookat[2] = 0.5;
    }

    void SetRealtimeFactor(double factor) {
        realtime_factor_ = std::max(0.01, std::min(10.0, factor));
        speed_changed_ = true;
    }

    double GetRealtimeFactor() const { return realtime_factor_; }

private:
    // MARK: - Private Methods (snake_case)

    void physics_loop() {
        printf("[MJRuntime] Physics loop started, instance %d, UDP port %d, server active: %s\n",
               instance_index_, udp_port_, udp_server_.IsActive() ? "YES" : "NO");

        Clock::time_point sync_cpu;
        mjtNum sync_sim = 0;

        int step_count = 0;
        int steps_since_last_frame = 0;
        auto last_stats_update = Clock::now();
        auto last_frame_write = Clock::now();
        int32_t current_sps = 0;

        // Frame write target: 80Hz (12.5ms interval)
        // Adaptive behavior:
        // - Physics >= 80Hz: write every ~12.5ms (skip some steps)
        // - Physics < 80Hz: write every step (can't exceed physics rate)
        constexpr double kTargetFrameWriteHz = 80.0;
        constexpr double kFrameWriteInterval = 1.0 / kTargetFrameWriteHz;

        // Temporary buffer for receiving control values
        std::vector<double> ctrl_buffer(256);  // Max 256 actuators

        while (!exit_requested_ && state_ == MJRuntimeStateRunning) {
            if (busy_wait_) {
                std::this_thread::yield();
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }

            if (!model_ || !data_) continue;

            // === UDP Control Loop ===
            // Drain the receive buffer: for each control packet, apply controls and step
            bool did_udp_step = false;
            if (udp_server_.IsActive()) {
                int nu = model_->nu;
                int max_packets = 100;  // Limit to prevent infinite loop
                int packets_processed = 0;

                while (packets_processed < max_packets) {
                    // Receive control packet - pass max buffer size, not model's nu
                    // This way we can receive packets even if model has no actuators
                    int received = udp_server_.ReceiveControl(ctrl_buffer.data(),
                                                              static_cast<int>(ctrl_buffer.size()));
                    if (received < 0) break;  // No more packets (0 is valid for models without actuators)

                    // Apply control values to mjData (only if model has actuators)
                    if (nu > 0 && received > 0) {
                        int copy_count = std::min(received, nu);
                        for (int i = 0; i < copy_count; i++) {
                            data_->ctrl[i] = ctrl_buffer[i];
                        }
                    }

                    // Step the simulation
                    mj_step(model_, data_);
                    step_count++;
                    steps_since_last_frame++;
                    did_udp_step = true;
                    packets_processed++;

                    // Send state back after each step
                    bool sent = udp_server_.SendState(model_, data_);
                    if (packets_processed <= 3) {
                        printf("[MJRuntime] UDP packet processed: received=%d, model_nu=%d, sent=%s\n",
                               received, nu, sent ? "YES" : "NO");
                    }
                }

                // Resync timing after UDP-driven steps
                if (did_udp_step) {
                    sync_cpu = Clock::now();
                    sync_sim = data_->time;
                    speed_changed_ = false;
                }
            }

            // === Real-time Physics Loop (when no UDP packets) ===
            // Only run real-time stepping if we didn't do any UDP-driven steps
            if (!did_udp_step) {
                bool stepped = false;
                const auto start_cpu = Clock::now();
                const auto elapsed_cpu = start_cpu - sync_cpu;
                double elapsed_sim = data_->time - sync_sim;
                double slowdown = 1.0 / realtime_factor_;

                bool misaligned = std::abs(Seconds(elapsed_cpu).count() / slowdown - elapsed_sim) > kSyncMisalign;

                if (elapsed_sim < 0 || elapsed_cpu.count() < 0 ||
                    sync_cpu.time_since_epoch().count() == 0 ||
                    misaligned || speed_changed_) {

                    sync_cpu = start_cpu;
                    sync_sim = data_->time;
                    speed_changed_ = false;

                    mj_step(model_, data_);
                    stepped = true;
                    step_count++;
                } else {
                    mjtNum prev_sim = data_->time;
                    double refresh_time = kSimRefreshFraction / target_fps_;

                    while (Seconds((data_->time - sync_sim) * slowdown) < Clock::now() - sync_cpu &&
                           Clock::now() - start_cpu < Seconds(refresh_time)) {

                        mj_step(model_, data_);
                        stepped = true;
                        step_count++;

                        if (data_->time < prev_sim) break;
                    }
                }

                if (stepped) {
                    steps_since_last_frame++;
                }
            }

            // Update stats (every second)
            auto now = Clock::now();
            auto stats_elapsed = Seconds(now - last_stats_update).count();
            if (stats_elapsed >= 1.0) {
                current_sps = static_cast<int32_t>(step_count / stats_elapsed);
                step_count = 0;
                last_stats_update = now;
            }

            // Adaptive frame writing:
            // - Time-based: at most 80Hz (don't overwhelm display)
            // - Step-based: at least 1 step occurred (have new data)
            // This naturally adapts to physics rate:
            //   1000Hz physics → write every ~12 steps → 80Hz frames
            //   200Hz physics  → write every ~2-3 steps → 80Hz frames
            //   100Hz physics  → write every ~1 step → ~100Hz frames (capped by physics)
            //   50Hz physics   → write every step → 50Hz frames (physics is bottleneck)
            auto frame_elapsed = Seconds(now - last_frame_write).count();
            if (steps_since_last_frame > 0 && frame_elapsed >= kFrameWriteInterval) {
                write_frame_to_buffer(current_sps);
                last_frame_write = now;
                steps_since_last_frame = 0;
            }
        }
    }

    void write_frame_to_buffer(int32_t sps = 0) {
        FrameData* frame = ring_buffer_.BeginWrite();

        // Update scene first (extracts transforms from mjData)
        mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

        // Copy geom data to frame buffer
        frame->geom_count = std::min(scene_.ngeom, kMaxGeoms);

        for (int i = 0; i < frame->geom_count; i++) {
            const mjvGeom& src = scene_.geoms[i];
            GeomInstance& dst = frame->geoms[i];

            // Position
            dst.pos[0] = src.pos[0];
            dst.pos[1] = src.pos[1];
            dst.pos[2] = src.pos[2];

            // Rotation matrix (3x3)
            for (int j = 0; j < 9; j++) {
                dst.mat[j] = src.mat[j];
            }

            // Size
            dst.size[0] = src.size[0];
            dst.size[1] = src.size[1];
            dst.size[2] = src.size[2];

            // Color
            dst.rgba[0] = src.rgba[0];
            dst.rgba[1] = src.rgba[1];
            dst.rgba[2] = src.rgba[2];
            dst.rgba[3] = src.rgba[3];

            // Properties
            dst.type = src.type;
            dst.emission = src.emission;
            dst.specular = src.specular;
            dst.shininess = src.shininess;
        }

        // Camera state
        frame->camera_azimuth = static_cast<float>(camera_.azimuth);
        frame->camera_elevation = static_cast<float>(camera_.elevation);
        frame->camera_distance = static_cast<float>(camera_.distance);
        frame->camera_lookat[0] = static_cast<float>(camera_.lookat[0]);
        frame->camera_lookat[1] = static_cast<float>(camera_.lookat[1]);
        frame->camera_lookat[2] = static_cast<float>(camera_.lookat[2]);

        // Compute camera world position from spherical coordinates
        float az_rad = static_cast<float>(camera_.azimuth * M_PI / 180.0);
        float el_rad = static_cast<float>(camera_.elevation * M_PI / 180.0);
        float dist = static_cast<float>(camera_.distance);
        frame->camera_pos[0] = frame->camera_lookat[0] + dist * sinf(az_rad) * cosf(el_rad);
        frame->camera_pos[1] = frame->camera_lookat[1] + dist * cosf(az_rad) * cosf(el_rad);
        frame->camera_pos[2] = frame->camera_lookat[2] + dist * sinf(el_rad);

        // Timing
        frame->simulation_time = data_->time;
        frame->steps_per_second = sps;
        frame->frame_number = ring_buffer_.GetFrameCount() + 1;

        // Publish frame (atomic, wakes up render thread)
        ring_buffer_.EndWrite();
    }

    // MARK: - Private Member Variables (snake_case)

    int32_t instance_index_;
    double target_fps_;
    bool busy_wait_;
    uint16_t udp_port_;
    std::atomic<double> realtime_factor_;
    std::atomic<MJRuntimeState> state_;

    mjModel* model_;
    mjData* data_;

    // Visualization (for mjv_updateScene)
    mjvScene scene_;
    mjvCamera camera_;
    mjvOption option_;

    // Lock-free ring buffer for render data
    LockFreeRingBuffer ring_buffer_;
    uint64_t last_read_frame_;

    // UDP server for network control
    UDPServer udp_server_;

    // Physics thread
    std::thread physics_thread_;
    std::atomic<bool> exit_requested_;
    std::atomic<bool> speed_changed_;
};

// MARK: - C Interface Implementation

extern "C" {

MJRuntimeHandle mj_runtime_create(const MJRuntimeConfig* config) {
    if (!config) return nullptr;
    try {
        return new MJSimulationRuntime(*config);
    } catch (...) {
        return nullptr;
    }
}

void mj_runtime_destroy(MJRuntimeHandle handle) {
    delete static_cast<MJSimulationRuntime*>(handle);
}

bool mj_runtime_load_model(MJRuntimeHandle handle,
                           const char* xmlPath,
                           char* errorBuffer,
                           int32_t errorBufferSize) {
    if (!handle || !xmlPath) return false;
    return static_cast<MJSimulationRuntime*>(handle)->LoadModel(xmlPath, errorBuffer, errorBufferSize);
}

bool mj_runtime_load_model_xml(MJRuntimeHandle handle,
                               const char* xmlString,
                               char* errorBuffer,
                               int32_t errorBufferSize) {
    if (!handle || !xmlString) return false;
    return static_cast<MJSimulationRuntime*>(handle)->LoadModelXML(xmlString, errorBuffer, errorBufferSize);
}

void mj_runtime_unload(MJRuntimeHandle handle) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->Unload();
}

void mj_runtime_start(MJRuntimeHandle handle) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->Start();
}

void mj_runtime_pause(MJRuntimeHandle handle) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->Pause();
}

void mj_runtime_reset(MJRuntimeHandle handle) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->Reset();
}

void mj_runtime_step(MJRuntimeHandle handle) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->Step();
}

MJRuntimeState mj_runtime_get_state(MJRuntimeHandle handle) {
    if (!handle) return MJRuntimeStateInactive;
    return static_cast<MJSimulationRuntime*>(handle)->GetState();
}

MJRuntimeStats mj_runtime_get_stats(MJRuntimeHandle handle) {
    if (!handle) return MJRuntimeStats{0, 1.0, 0.002, 0};
    return static_cast<MJSimulationRuntime*>(handle)->GetStats();
}

// Legacy scene access (for compatibility)
void mj_runtime_lock(MJRuntimeHandle handle) {
    // No-op with ring buffer - kept for API compatibility
}

void mj_runtime_unlock(MJRuntimeHandle handle) {
    // No-op with ring buffer - kept for API compatibility
}

void mj_runtime_update_scene(MJRuntimeHandle handle) {
    // No-op - scene is updated in ring buffer automatically
}

const mjvScene* mj_runtime_get_scene(MJRuntimeHandle handle) {
    if (!handle) return nullptr;
    return static_cast<MJSimulationRuntime*>(handle)->GetScene();
}

mjvCamera* mj_runtime_get_camera(MJRuntimeHandle handle) {
    if (!handle) return nullptr;
    return static_cast<MJSimulationRuntime*>(handle)->GetCamera();
}

mjvOption* mj_runtime_get_option(MJRuntimeHandle handle) {
    if (!handle) return nullptr;
    return static_cast<MJSimulationRuntime*>(handle)->GetOption();
}

const mjModel* mj_runtime_get_model(MJRuntimeHandle handle) {
    if (!handle) return nullptr;
    return static_cast<MJSimulationRuntime*>(handle)->GetModel();
}

const mjData* mj_runtime_get_data(MJRuntimeHandle handle) {
    if (!handle) return nullptr;
    return static_cast<MJSimulationRuntime*>(handle)->GetData();
}

// New ring buffer API
const MJFrameData* mj_runtime_wait_for_frame(MJRuntimeHandle handle) {
    if (!handle) return nullptr;
    // Cast internal FrameData to MJFrameData (same memory layout)
    return reinterpret_cast<const MJFrameData*>(
        static_cast<MJSimulationRuntime*>(handle)->WaitForFrame());
}

const MJFrameData* mj_runtime_get_latest_frame(MJRuntimeHandle handle) {
    if (!handle) return nullptr;
    // Cast internal FrameData to MJFrameData (same memory layout)
    return reinterpret_cast<const MJFrameData*>(
        static_cast<MJSimulationRuntime*>(handle)->GetLatestFrame());
}

uint64_t mj_runtime_get_frame_count(MJRuntimeHandle handle) {
    if (!handle) return 0;
    return static_cast<MJSimulationRuntime*>(handle)->GetFrameCount();
}

// Frame data accessors (for Swift interop with large fixed-size arrays)
const MJGeomInstance* mj_frame_get_geoms(const MJFrameData* frame) {
    if (!frame) return nullptr;
    // The geoms array in MJFrameData starts at offset 0
    return frame->geoms;
}

int32_t mj_frame_get_geom_count(const MJFrameData* frame) {
    if (!frame) return 0;
    return frame->geom_count;
}

const MJGeomInstance* mj_frame_get_geom(const MJFrameData* frame, int32_t index) {
    if (!frame || index < 0 || index >= frame->geom_count) return nullptr;
    return &frame->geoms[index];
}

// Camera control
void mj_runtime_set_camera_azimuth(MJRuntimeHandle handle, double azimuth) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->SetCameraAzimuth(azimuth);
}

void mj_runtime_set_camera_elevation(MJRuntimeHandle handle, double elevation) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->SetCameraElevation(elevation);
}

void mj_runtime_set_camera_distance(MJRuntimeHandle handle, double distance) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->SetCameraDistance(distance);
}

void mj_runtime_set_camera_lookat(MJRuntimeHandle handle, double x, double y, double z) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->SetCameraLookat(x, y, z);
}

void mj_runtime_reset_camera(MJRuntimeHandle handle) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->ResetCamera();
}

void mj_runtime_set_realtime_factor(MJRuntimeHandle handle, double factor) {
    if (handle) static_cast<MJSimulationRuntime*>(handle)->SetRealtimeFactor(factor);
}

double mj_runtime_get_realtime_factor(MJRuntimeHandle handle) {
    if (!handle) return 1.0;
    return static_cast<MJSimulationRuntime*>(handle)->GetRealtimeFactor();
}

}  // extern "C"
