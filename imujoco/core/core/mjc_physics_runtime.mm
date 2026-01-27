// mjc_physics_runtime.mm
// C++ implementation of MuJoCo physics simulation runtime
// Uses SpscQueue (single-producer multi-reader "latest value" queue) with C++20 atomic
// wait/notify for lowest latency. Multiple threads may call WaitForFrame() concurrently.

#include "mjc_physics_runtime.h"
#include <mujoco/mujoco.h>

#include <algorithm>
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

// Apple unified logging
#include <os/log.h>

// FlatBuffers serialization
#include <flatbuffers/flatbuffers.h>
#include "imujoco/schema/state_generated.h"
#include "imujoco/schema/control_generated.h"

// Fragment support
#include "mjc_fragment.h"

// SPSC queue for lock-free frame passing
#include "spsc_queue.h"

// MARK: - Constants

namespace {

constexpr std::size_t kFrameQueueCapacity = 3;  // Triple buffering for frame data
constexpr int kErrorLength = 1024;              // Error buffer size
constexpr int kMaxPacketsPerLoop = 100;         // Max UDP packets to process per physics step
constexpr int kMaxActuators = 256;              // Maximum actuators for control buffer

// Physics timing constants
constexpr double kSyncMisalign = 0.1;          // Maximum acceptable drift
constexpr double kSimRefreshFraction = 0.7;   // Fraction of timestep to refresh

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

}  // namespace

// MARK: - UDPServer

class UDPServer {
public:
    UDPServer() : socket_fd_(-1), port_(0), has_client_(false),
                  packets_received_(0), packets_sent_(0), send_sequence_(0),
                  fb_builder_(4096) {}

    ~UDPServer() { Close(); }

    bool Start(uint16_t port) {
        if (port == 0) return false;

        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            os_log_error(OS_LOG_DEFAULT, "Failed to create socket");
            return false;
        }

        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            os_log_error(OS_LOG_DEFAULT, "Failed to bind to port %u", port);
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        port_ = port;
        os_log_info(OS_LOG_DEFAULT, "Listening on port %u", port);
        return true;
    }

    void Close() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
            os_log_info(OS_LOG_DEFAULT, "Closed port %u", port_);
        }
        port_ = 0;
        has_client_ = false;
    }

    bool IsActive() const { return socket_fd_ >= 0; }
    uint16_t GetPort() const { return port_; }
    bool HasClient() const { return has_client_; }
    uint32_t GetPacketsReceived() const { return packets_received_; }
    uint32_t GetPacketsSent() const { return packets_sent_; }

    int ReceiveControl(double* ctrl_out, int max_ctrl) {
        if (socket_fd_ < 0) return -1;

        uint8_t buffer[MJ_MAX_UDP_PAYLOAD];
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                    (struct sockaddr*)&client_addr, &addr_len);

        if (recv_len <= 0) return -1;

        client_addr_ = client_addr;
        has_client_ = true;

        os_log_debug(OS_LOG_DEFAULT, "Received %zd bytes", recv_len);

        reassembly_manager_.CleanupStale();

        if (recv_len >= static_cast<ssize_t>(MJ_FRAGMENT_HEADER_SIZE)) {
            const MJFragmentHeader* frag_header = reinterpret_cast<const MJFragmentHeader*>(buffer);
            if (frag_header->magic == MJ_PACKET_MAGIC_FRAG) {
                size_t message_size = 0;
                const uint8_t* complete = reassembly_manager_.ProcessFragment(
                    buffer, static_cast<size_t>(recv_len), &message_size);

                if (!complete) return -1;

                reassembly_buffer_.resize(message_size);
                std::memcpy(reassembly_buffer_.data(), complete, message_size);

                return ParseControlPacket(reassembly_buffer_.data(), message_size, ctrl_out, max_ctrl);
            }
        }

        return ParseControlPacket(buffer, static_cast<size_t>(recv_len), ctrl_out, max_ctrl);
    }

    int ParseControlPacket(const uint8_t* data, size_t size, double* ctrl_out, int max_ctrl) {
        flatbuffers::Verifier verifier(data, size);
        if (!imujoco::schema::VerifyControlPacketBuffer(verifier)) {
            os_log_error(OS_LOG_DEFAULT, "Invalid FlatBuffers ControlPacket buffer");
            return -1;
        }

        auto packet = imujoco::schema::GetControlPacket(data);
        packets_received_++;

        os_log_debug(OS_LOG_DEFAULT, "ControlPacket: seq=%u", packet->sequence());

        auto ctrl = packet->ctrl();
        if (!ctrl || ctrl->size() == 0) return 0;

        int nu = static_cast<int>(ctrl->size());
        int copy_count = std::min(nu, max_ctrl);

        if (copy_count > 0 && ctrl_out != nullptr) {
            for (int i = 0; i < copy_count; i++) {
                ctrl_out[i] = ctrl->Get(i);
            }
        }

        return copy_count;
    }

    bool SendState(const mjModel* model, const mjData* data) {
        if (socket_fd_ < 0 || !has_client_ || !model || !data) {
            if (socket_fd_ < 0) os_log_debug(OS_LOG_DEFAULT, "SendState: socket not open");
            else if (!has_client_) os_log_debug(OS_LOG_DEFAULT, "SendState: no client");
            return false;
        }

        fb_builder_.Clear();

        flatbuffers::Offset<flatbuffers::Vector<double>> qpos_vec = 0;
        flatbuffers::Offset<flatbuffers::Vector<double>> qvel_vec = 0;
        flatbuffers::Offset<flatbuffers::Vector<double>> ctrl_vec = 0;
        flatbuffers::Offset<flatbuffers::Vector<double>> sensor_vec = 0;

        if (model->nq > 0) qpos_vec = fb_builder_.CreateVector(data->qpos, model->nq);
        if (model->nv > 0) qvel_vec = fb_builder_.CreateVector(data->qvel, model->nv);
        if (model->nu > 0) ctrl_vec = fb_builder_.CreateVector(data->ctrl, model->nu);
        if (model->nsensordata > 0) sensor_vec = fb_builder_.CreateVector(data->sensordata, model->nsensordata);

        auto state_packet = imujoco::schema::CreateStatePacket(
            fb_builder_, send_sequence_++, data->time,
            data->energy[0], data->energy[1],
            qpos_vec, qvel_vec, ctrl_vec, sensor_vec
        );

        imujoco::schema::FinishStatePacketBuffer(fb_builder_, state_packet);

        int fragments = fragmented_sender_.SendMessage(
            fb_builder_.GetBufferPointer(), fb_builder_.GetSize(),
            socket_fd_, client_addr_
        );

        if (fragments > 0) {
            packets_sent_++;
            if (packets_sent_ <= 5 || packets_sent_ % 100 == 0) {
                os_log_debug(OS_LOG_DEFAULT, "Sent state packet #%u (%u bytes, %d fragments)",
                             packets_sent_, static_cast<uint32_t>(fb_builder_.GetSize()), fragments);
            }
            return true;
        }

        os_log_error(OS_LOG_DEFAULT, "Failed to send state packet");
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

    flatbuffers::FlatBufferBuilder fb_builder_;
    imujoco::FragmentedSender fragmented_sender_;
    imujoco::ReassemblyManager reassembly_manager_;
    std::vector<uint8_t> reassembly_buffer_;
};

// MARK: - MJSimulationRuntimeImpl

// Monotonic counter for generating unique instance IDs that survive object reuse.
// Used by WaitForFrame() to maintain per-thread-per-instance state.
static std::atomic<uint64_t> g_next_instance_id{1};

class MJSimulationRuntimeImpl {
public:
    explicit MJSimulationRuntimeImpl(const MJRuntimeConfig& config)
        : unique_id_(g_next_instance_id.fetch_add(1, std::memory_order_relaxed))
        , instance_index_(config.instanceIndex)
        , target_fps_(config.targetFPS > 0 ? config.targetFPS : 60.0)
        , busy_wait_(config.busyWait)
        , udp_port_(config.udpPort > 0 ? config.udpPort : MJ_DEFAULT_UDP_PORT + config.instanceIndex)
        , realtime_factor_(1.0)
        , state_(MJRuntimeState::Inactive)
        , model_(nullptr)
        , data_(nullptr)
        , exit_requested_(false)
        , speed_changed_(false) {

        os_log_info(OS_LOG_DEFAULT, "Creating instance %d (SPSC queue, UDP port %u)",
                    config.instanceIndex, udp_port_);

        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&option_);

        memset(&scene_, 0, sizeof(mjvScene));
        mjv_makeScene(nullptr, &scene_, MJ_MAX_GEOMS);

        camera_.type = mjCAMERA_FREE;
        camera_.azimuth = 90;
        camera_.elevation = -15;
        camera_.distance = 3.0;
        camera_.lookat[0] = 0;
        camera_.lookat[1] = 0;
        camera_.lookat[2] = 0.5;

        os_log_info(OS_LOG_DEFAULT, "Instance %d ready", config.instanceIndex);
    }

    ~MJSimulationRuntimeImpl() {
        os_log_info(OS_LOG_DEFAULT, "Destroying instance %d", instance_index_);
        Stop();
        Unload();
        if (scene_.maxgeom > 0) {
            mjv_freeScene(&scene_);
        }
        os_log_info(OS_LOG_DEFAULT, "Instance %d destroyed", instance_index_);
    }

    bool LoadModel(const char* xml_path, std::string* out_error) {
        os_log_info(OS_LOG_DEFAULT, "LoadModel: %{public}s", xml_path ? xml_path : "(null)");
        Stop();

        if (data_) { mj_deleteData(data_); data_ = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }

        char load_error[kErrorLength] = "";
        mjModel* mnew = mj_loadXML(xml_path, nullptr, load_error, kErrorLength);

        if (!mnew) {
            if (out_error) *out_error = load_error;
            state_ = MJRuntimeState::Inactive;
            return false;
        }

        mjData* dnew = mj_makeData(mnew);
        if (!dnew) {
            mj_deleteModel(mnew);
            if (out_error) *out_error = "Failed to create mjData";
            state_ = MJRuntimeState::Inactive;
            return false;
        }

        model_ = mnew;
        data_ = dnew;

        if (model_->nkey > 0) {
            mj_resetDataKeyframe(model_, data_, 0);
        }

        mj_forward(model_, data_);
        WriteFrameToBuffer();

        state_ = MJRuntimeState::Loaded;
        return true;
    }

    bool LoadModelXML(const char* xml_string, std::string* out_error) {
        Stop();

        if (data_) { mj_deleteData(data_); data_ = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }

        char load_error[kErrorLength] = "";
        mjSpec* spec = mj_parseXMLString(xml_string, nullptr, load_error, kErrorLength);
        if (!spec) {
            if (out_error) *out_error = load_error;
            state_ = MJRuntimeState::Inactive;
            return false;
        }

        mjModel* mnew = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);

        if (!mnew) {
            if (out_error) *out_error = "Failed to compile model";
            state_ = MJRuntimeState::Inactive;
            return false;
        }

        mjData* dnew = mj_makeData(mnew);
        if (!dnew) {
            mj_deleteModel(mnew);
            if (out_error) *out_error = "Failed to create mjData";
            state_ = MJRuntimeState::Inactive;
            return false;
        }

        model_ = mnew;
        data_ = dnew;

        if (model_->nkey > 0) {
            mj_resetDataKeyframe(model_, data_, 0);
        }

        mj_forward(model_, data_);
        WriteFrameToBuffer();

        state_ = MJRuntimeState::Loaded;
        return true;
    }

    void Unload() {
        Stop();

        if (data_) { mj_deleteData(data_); data_ = nullptr; }
        if (model_) { mj_deleteModel(model_); model_ = nullptr; }

        state_ = MJRuntimeState::Inactive;
    }

    void Start() {
        if (!model_ || !data_) return;
        if (state_ == MJRuntimeState::Running) return;

        if (udp_port_ > 0 && !udp_server_.IsActive()) {
            udp_server_.Start(udp_port_);
        }

        state_ = MJRuntimeState::Running;
        exit_requested_ = false;
        speed_changed_ = true;
        // IMPORTANT: reset_exit_signal() is only safe if no threads are blocked in
        // WaitForFrame(). The caller must ensure all WaitForFrame() calls have returned
        // (received nullptr from the previous Stop()) before calling Start() again.
        // This is typically ensured by the application's shutdown/restart sequence.
        ring_buffer_.reset_exit_signal();

        physics_thread_ = std::thread(&MJSimulationRuntimeImpl::PhysicsLoop, this);
    }

    void Pause() {
        if (state_ != MJRuntimeState::Running) return;
        state_ = MJRuntimeState::Paused;
        Stop();
    }

    void Stop() {
        exit_requested_ = true;
        ring_buffer_.signal_exit();

        if (physics_thread_.joinable()) {
            physics_thread_.join();
        }

        udp_server_.Close();
    }

    void Reset() {
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
        if (model_ && data_ && state_ != MJRuntimeState::Running) {
            mj_step(model_, data_);
            WriteFrameToBuffer();
        }
    }

    MJRuntimeState GetState() const { return state_; }

    MJRuntimeStats GetStats() const {
        const MJFrameDataStorage* storage = ring_buffer_.get_latest();
        MJRuntimeStats stats;
        stats.simulationTime = storage ? storage->simulationTime : 0.0;
        stats.measuredSlowdown = 1.0;
        stats.timestep = model_ ? model_->opt.timestep : 0.002;
        stats.stepsPerSecond = storage ? storage->stepsPerSecond : 0;
        stats.udpPort = udp_server_.IsActive() ? udp_server_.GetPort() : 0;
        stats.packetsReceived = udp_server_.GetPacketsReceived();
        stats.packetsSent = udp_server_.GetPacketsSent();
        stats.hasClient = udp_server_.HasClient();
        return stats;
    }

    // Helper: allocate MJFrameData view from shared thread-local pool.
    //
    // Design: Pool of 2 views provides breathing room for callers that need to
    // briefly hold a reference while calling getLatestFrame() again (e.g., for
    // comparison). Only the most recent pointer is guaranteed valid; the previous
    // one becomes invalid on the next call.
    //
    // Why not use assignment/reuse? MJFrameData is non-copyable (deleted copy
    // constructor/assignment) to enforce reference semantics in Swift. We must
    // create new instances rather than mutate existing ones.
    //
    // Alternatives considered:
    // - Single instance: Too restrictive for any compare-and-swap patterns
    // - Larger pool: Diminishing returns, 2 is sufficient for typical use
    // - shared_ptr: Would require API changes and add overhead
    //
    // The lifetime constraint is documented in mjc_physics_runtime.h (lines 116-124).
    static MJFrameData* AllocateFrameView(const MJFrameDataStorage* storage) {
        if (!storage) return nullptr;
        // Thread-local pool shared by GetLatestFrame() and WaitForFrame()
        thread_local std::array<std::unique_ptr<MJFrameData>, 2> tls_view_pool;
        thread_local size_t tls_view_index = 0;
        tls_view_index = (tls_view_index + 1) % tls_view_pool.size();
        tls_view_pool[tls_view_index] = std::make_unique<MJFrameData>(storage);
        return tls_view_pool[tls_view_index].get();
    }

    MJFrameData* GetLatestFrame() {
        const MJFrameDataStorage* storage = ring_buffer_.get_latest();
        return AllocateFrameView(storage);
    }

    MJFrameData* WaitForFrame() {
        // Per-thread-per-instance state for tracking last item count.
        // Uses a fixed-capacity array for allocation-free operation on real-time threads.
        // Capacity of 4 covers typical usage (1-2 instances per thread); excess entries
        // are evicted LRU-style with their state reset on next access.
        static constexpr size_t kMaxCachedInstances = 4;
        struct CacheEntry {
            uint64_t id = 0;
            uint64_t count = 0;
        };
        struct PerThreadState {
            std::array<CacheEntry, kMaxCachedInstances> entries{};
            size_t size = 0;  // Number of valid entries
        };
        thread_local PerThreadState state;

        // Linear search for this instance (fast for small N)
        uint64_t* last_ptr = nullptr;
        size_t found_idx = state.size;
        for (size_t i = 0; i < state.size; ++i) {
            if (state.entries[i].id == unique_id_) {
                found_idx = i;
                last_ptr = &state.entries[i].count;
                break;
            }
        }

        if (!last_ptr) {
            // Cache miss: need to add or evict
            if (state.size < kMaxCachedInstances) {
                // Space available: append
                found_idx = state.size++;
            } else {
                // Full: evict oldest (index 0), shift entries, append at end
                // This maintains LRU order: most recently used at the end
                for (size_t i = 0; i + 1 < kMaxCachedInstances; ++i) {
                    state.entries[i] = state.entries[i + 1];
                }
                found_idx = kMaxCachedInstances - 1;
            }
            state.entries[found_idx].id = unique_id_;
            // Initialize to current item_count so wait_for_item blocks until a NEW frame.
            // This handles re-access after eviction correctly (won't return stale frame).
            state.entries[found_idx].count = ring_buffer_.get_item_count();
            last_ptr = &state.entries[found_idx].count;
        } else if (found_idx + 1 < state.size) {
            // Move-to-back for LRU: swap this entry toward the end
            CacheEntry tmp = state.entries[found_idx];
            for (size_t i = found_idx; i + 1 < state.size; ++i) {
                state.entries[i] = state.entries[i + 1];
            }
            state.entries[state.size - 1] = tmp;
            last_ptr = &state.entries[state.size - 1].count;
        }

        const MJFrameDataStorage* storage = ring_buffer_.wait_for_item(*last_ptr);
        *last_ptr = ring_buffer_.get_item_count();
        return AllocateFrameView(storage);
    }

    uint64_t GetFrameCount() const {
        // Return the true number of frames produced, excluding exit signals.
        return ring_buffer_.get_item_count();
    }

    // Camera
    void SetCameraAzimuth(double azimuth) { camera_.azimuth = azimuth; }
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

    double GetCameraAzimuth() const { return camera_.azimuth; }
    double GetCameraElevation() const { return camera_.elevation; }
    double GetCameraDistance() const { return camera_.distance; }

    void SetRealtimeFactor(double factor) {
        realtime_factor_ = std::max(0.01, std::min(10.0, factor));
        speed_changed_ = true;
    }
    double GetRealtimeFactor() const { return realtime_factor_; }

    // Legacy access
    const mjvScene* GetScene() const { return &scene_; }
    mjvCamera* GetCamera() { return &camera_; }
    mjvOption* GetOption() { return &option_; }
    const mjModel* GetModel() const { return model_; }
    const mjData* GetData() const { return data_; }

private:
    void PhysicsLoop() {
        os_log_info(OS_LOG_DEFAULT, "Physics loop started, instance %d, UDP port %u, server active: %s",
                    instance_index_, udp_port_, udp_server_.IsActive() ? "YES" : "NO");

        Clock::time_point sync_cpu;
        mjtNum sync_sim = 0;

        int step_count = 0;
        int steps_since_last_frame = 0;
        auto last_stats_update = Clock::now();
        auto last_frame_write = Clock::now();
        int32_t current_sps = 0;

        constexpr double kTargetFrameWriteHz = 80.0;
        constexpr double kFrameWriteInterval = 1.0 / kTargetFrameWriteHz;

        std::vector<double> ctrl_buffer(kMaxActuators);

        while (!exit_requested_ && state_ == MJRuntimeState::Running) {
            if (busy_wait_) {
                std::this_thread::yield();
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }

            if (!model_ || !data_) continue;

            // === UDP Control Loop ===
            bool did_udp_step = false;
            if (udp_server_.IsActive()) {
                int nu = model_->nu;
                int packets_processed = 0;

                while (packets_processed < kMaxPacketsPerLoop) {
                    int received = udp_server_.ReceiveControl(ctrl_buffer.data(),
                                                              static_cast<int>(ctrl_buffer.size()));
                    if (received < 0) break;

                    if (nu > 0 && received > 0) {
                        int copy_count = std::min(received, nu);
                        for (int i = 0; i < copy_count; i++) {
                            data_->ctrl[i] = ctrl_buffer[i];
                        }
                    }

                    mj_step(model_, data_);
                    step_count++;
                    steps_since_last_frame++;
                    did_udp_step = true;
                    packets_processed++;

                    bool sent = udp_server_.SendState(model_, data_);
                    if (packets_processed <= 3) {
                        os_log_debug(OS_LOG_DEFAULT, "UDP packet processed: received=%d, model_nu=%d, sent=%s",
                                     received, nu, sent ? "YES" : "NO");
                    }
                }

                if (did_udp_step) {
                    sync_cpu = Clock::now();
                    sync_sim = data_->time;
                    speed_changed_ = false;
                }
            }

            // === Real-time Physics Loop (when no UDP packets) ===
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

                        if (data_->time < prev_sim) {
                            os_log_info(OS_LOG_DEFAULT, "Warning: simulation time went backwards, resyncing");
                            speed_changed_ = true;
                            break;
                        }
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

            // Adaptive frame writing
            auto frame_elapsed = Seconds(now - last_frame_write).count();
            if (steps_since_last_frame > 0 && frame_elapsed >= kFrameWriteInterval) {
                WriteFrameToBuffer(current_sps);
                last_frame_write = now;
                steps_since_last_frame = 0;
            }
        }
    }

    void WriteFrameToBuffer(int32_t sps = 0) {
        MJFrameDataStorage* frame = ring_buffer_.begin_write();

        mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

        frame->geomCount = std::min(scene_.ngeom, MJ_MAX_GEOMS);

        for (int i = 0; i < frame->geomCount; i++) {
            const mjvGeom& src = scene_.geoms[i];
            MJGeomInstance& dst = frame->geoms[i];

            dst.pos[0] = src.pos[0];
            dst.pos[1] = src.pos[1];
            dst.pos[2] = src.pos[2];

            for (int j = 0; j < 9; j++) {
                dst.mat[j] = src.mat[j];
            }

            dst.size[0] = src.size[0];
            dst.size[1] = src.size[1];
            dst.size[2] = src.size[2];

            dst.rgba[0] = src.rgba[0];
            dst.rgba[1] = src.rgba[1];
            dst.rgba[2] = src.rgba[2];
            dst.rgba[3] = src.rgba[3];

            dst.type = src.type;
            dst.emission = src.emission;
            dst.specular = src.specular;
            dst.shininess = src.shininess;
        }

        frame->cameraAzimuth = static_cast<float>(camera_.azimuth);
        frame->cameraElevation = static_cast<float>(camera_.elevation);
        frame->cameraDistance = static_cast<float>(camera_.distance);
        frame->cameraLookat[0] = static_cast<float>(camera_.lookat[0]);
        frame->cameraLookat[1] = static_cast<float>(camera_.lookat[1]);
        frame->cameraLookat[2] = static_cast<float>(camera_.lookat[2]);

        double az_rad = camera_.azimuth * M_PI / 180.0;
        double el_rad = camera_.elevation * M_PI / 180.0;
        double dist = camera_.distance;
        frame->cameraPos[0] = static_cast<float>(camera_.lookat[0] + dist * sin(az_rad) * cos(el_rad));
        frame->cameraPos[1] = static_cast<float>(camera_.lookat[1] + dist * cos(az_rad) * cos(el_rad));
        frame->cameraPos[2] = static_cast<float>(camera_.lookat[2] + dist * sin(el_rad));

        frame->simulationTime = data_->time;
        frame->stepsPerSecond = sps;
        // Use get_item_count() for actual frame count (not affected by signal_exit())
        frame->frameNumber = ring_buffer_.get_item_count() + 1;

        ring_buffer_.end_write();
    }

    const uint64_t unique_id_;  // Monotonic ID for per-thread state keying (survives address reuse)
    int32_t instance_index_;
    double target_fps_;
    bool busy_wait_;
    uint16_t udp_port_;
    std::atomic<double> realtime_factor_;
    std::atomic<MJRuntimeState> state_;

    mjModel* model_;
    mjData* data_;

    mjvScene scene_;
    mjvCamera camera_;
    mjvOption option_;

    imujoco::SpscQueue<MJFrameDataStorage, kFrameQueueCapacity> ring_buffer_;
    UDPServer udp_server_;

    std::thread physics_thread_;
    std::atomic<bool> exit_requested_;
    std::atomic<bool> speed_changed_;
};

// MARK: - Version Info

int32_t MJGetVersion() {
    return mj_version();
}

// MARK: - MJFrameData Free Functions

const MJGeomInstance* MJFrameDataGetGeoms(const MJFrameData* frame) {
    if (!frame || !frame->storage_) return nullptr;
    return frame->storage_->geoms;
}

// MARK: - MJSimulationRuntime Public Interface

MJSimulationRuntime::MJSimulationRuntime(std::unique_ptr<MJSimulationRuntimeImpl> impl)
    : impl_(std::move(impl)) {}

MJSimulationRuntime::~MJSimulationRuntime() = default;

MJSimulationRuntime* MJSimulationRuntime::create(const MJRuntimeConfig& config) {
    try {
        auto impl = std::make_unique<MJSimulationRuntimeImpl>(config);
        return new MJSimulationRuntime(std::move(impl));
    } catch (const std::exception& e) {
        os_log_error(OS_LOG_DEFAULT, "MJSimulationRuntime::create failed: %{public}s", e.what());
        return nullptr;
    } catch (...) {
        os_log_error(OS_LOG_DEFAULT, "MJSimulationRuntime::create failed with unknown exception");
        return nullptr;
    }
}

void MJSimulationRuntime::destroy(MJSimulationRuntime* runtime) {
    if (runtime) {
        delete runtime;
    }
}

bool MJSimulationRuntime::loadModel(const char* xmlPath, std::string* outError) {
    return impl_->LoadModel(xmlPath, outError);
}

bool MJSimulationRuntime::loadModelXML(const char* xmlString, std::string* outError) {
    return impl_->LoadModelXML(xmlString, outError);
}

void MJSimulationRuntime::unload() { impl_->Unload(); }
void MJSimulationRuntime::start() { impl_->Start(); }
void MJSimulationRuntime::pause() { impl_->Pause(); }
void MJSimulationRuntime::reset() { impl_->Reset(); }
void MJSimulationRuntime::step() { impl_->Step(); }

MJRuntimeState MJSimulationRuntime::getState() const { return impl_->GetState(); }
MJRuntimeStats MJSimulationRuntime::getStats() const { return impl_->GetStats(); }

MJFrameData* MJSimulationRuntime::getLatestFrame() { return impl_->GetLatestFrame(); }
MJFrameData* MJSimulationRuntime::waitForFrame() { return impl_->WaitForFrame(); }
uint64_t MJSimulationRuntime::getFrameCount() const { return impl_->GetFrameCount(); }

void MJSimulationRuntime::setCameraAzimuth(double v) { impl_->SetCameraAzimuth(v); }
void MJSimulationRuntime::setCameraElevation(double v) { impl_->SetCameraElevation(v); }
void MJSimulationRuntime::setCameraDistance(double v) { impl_->SetCameraDistance(v); }
void MJSimulationRuntime::setCameraLookat(double x, double y, double z) { impl_->SetCameraLookat(x, y, z); }
void MJSimulationRuntime::resetCamera() { impl_->ResetCamera(); }

double MJSimulationRuntime::getCameraAzimuth() const { return impl_->GetCameraAzimuth(); }
double MJSimulationRuntime::getCameraElevation() const { return impl_->GetCameraElevation(); }
double MJSimulationRuntime::getCameraDistance() const { return impl_->GetCameraDistance(); }

void MJSimulationRuntime::setRealtimeFactor(double v) { impl_->SetRealtimeFactor(v); }
double MJSimulationRuntime::getRealtimeFactor() const { return impl_->GetRealtimeFactor(); }

const mjvScene* MJSimulationRuntime::getScene() const { return impl_->GetScene(); }
mjvCamera* MJSimulationRuntime::getCamera() { return impl_->GetCamera(); }
mjvOption* MJSimulationRuntime::getOption() { return impl_->GetOption(); }
const mjModel* MJSimulationRuntime::getModel() const { return impl_->GetModel(); }
const mjData* MJSimulationRuntime::getData() const { return impl_->GetData(); }
