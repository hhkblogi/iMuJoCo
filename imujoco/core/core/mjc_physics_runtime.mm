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
#include <deque>
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

// SPSC queue for mutex-free frame passing (producer is wait-free; consumers may block)
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

    int ReceiveControl(double* ctrl_out, int max_ctrl, uint64_t* host_timestamp_us_out) {
        if (socket_fd_ < 0) return -1;

        uint8_t buffer[MJ_MAX_UDP_PAYLOAD];
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                    (struct sockaddr*)&client_addr, &addr_len);

        if (recv_len <= 0) return -1;

        client_addr_ = client_addr;
        has_client_ = true;

        // Hot-path: skip per-packet logging

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

                return ParseControlPacket(reassembly_buffer_.data(), message_size,
                                          ctrl_out, max_ctrl, host_timestamp_us_out);
            }
        }

        return ParseControlPacket(buffer, static_cast<size_t>(recv_len),
                                  ctrl_out, max_ctrl, host_timestamp_us_out);
    }

    int ParseControlPacket(const uint8_t* data, size_t size,
                           double* ctrl_out, int max_ctrl,
                           uint64_t* host_timestamp_us_out) {
        flatbuffers::Verifier verifier(data, size);
        if (!imujoco::schema::VerifyControlPacketBuffer(verifier)) {
            os_log_error(OS_LOG_DEFAULT, "Invalid FlatBuffers ControlPacket buffer");
            return -1;
        }

        auto packet = imujoco::schema::GetControlPacket(data);
        packets_received_++;

        // Hot-path: skip per-packet logging

        if (host_timestamp_us_out) {
            *host_timestamp_us_out = packet->host_timestamp_us();
        }

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
        , ctrl_timeout_ms_(config.ctrlTimeoutMs)
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
        camera_.azimuth = 60;
        camera_.elevation = 45;
        camera_.distance = 4.0;
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
            int keyId = mj_name2id(model_, mjOBJ_KEY, "supine");
            mj_resetDataKeyframe(model_, data_, keyId >= 0 ? keyId : 0);
        }

        mj_forward(model_, data_);
        BuildActuatorTimeoutPolicy();
        WriteFrameToBuffer();
        ExtractMeshData();

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
            int keyId = mj_name2id(model_, mjOBJ_KEY, "supine");
            mj_resetDataKeyframe(model_, data_, keyId >= 0 ? keyId : 0);
        }

        mj_forward(model_, data_);
        BuildActuatorTimeoutPolicy();
        WriteFrameToBuffer();
        ExtractMeshData();

        state_ = MJRuntimeState::Loaded;
        return true;
    }

    void Unload() {
        Stop();

        mesh_data_.reset();
        mesh_storage_.reset();

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
        // Safe to reset: any threads still blocked in WaitForFrame() from the
        // previous run carry a cancellation predicate that checks the epoch
        // counter. reset_exit_signal() wakes them, and the epoch change
        // (advanced in Stop()) causes the predicate to return true, so they
        // return nullptr without re-blocking.
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
        // Advance epoch so stale WaitForFrame() callers from this run
        // detect the generation change and return nullptr.
        epoch_.fetch_add(1, std::memory_order_release);

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
            // Clear replay state
            ctrl_queue_.clear();
            replay_anchor_host_us_ = 0;
            replay_anchor_cpu_ = Clock::time_point{};
            ctrl_timed_out_ = false;
            last_ctrl_received_ = Clock::time_point{};
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
        stats.stepsPerSecondF = storage ? storage->stepsPerSecondF : 0.0f;
        stats.txRate = storage ? storage->txRate : 0.0f;
        stats.rxRate = storage ? storage->rxRate : 0.0f;
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
            // Initialize to (item_count - 1) so first call returns the latest existing frame
            // immediately (preserving original semantics). For re-access after eviction,
            // this returns the current latest frame which is acceptable.
            uint64_t current = ring_buffer_.get_item_count();
            state.entries[found_idx].count = (current > 0) ? (current - 1) : 0;
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

        // Capture epoch before blocking. If Stop()/Start() cycles while we
        // wait, the epoch will advance and we return nullptr instead of
        // delivering a frame from a different run.
        uint64_t epoch_before = epoch_.load(std::memory_order_acquire);

        const MJFrameDataStorage* storage = ring_buffer_.wait_for_item(
            *last_ptr,
            [&] { return epoch_.load(std::memory_order_acquire) != epoch_before; });

        // Check for stale wakeup from a previous run.
        if (epoch_.load(std::memory_order_acquire) != epoch_before) {
            return nullptr;
        }
        if (storage) {
            // Use the item's frameNumber to avoid race with producer.
            // get_item_count() could race ahead if producer writes between
            // wait_for_item() returning and this load.
            *last_ptr = storage->frameNumber;
        }
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
        camera_.azimuth = 60;
        camera_.elevation = 45;
        camera_.distance = 4.0;
        camera_.lookat[0] = 0;
        camera_.lookat[1] = 0;
        camera_.lookat[2] = 0.5;
    }

    double GetCameraAzimuth() const { return camera_.azimuth; }
    double GetCameraElevation() const { return camera_.elevation; }
    double GetCameraDistance() const { return camera_.distance; }

    void SetTimestep(double ts) {
        if (!model_ || ts <= 0.0 || !std::isfinite(ts)) return;
        model_->opt.timestep = ts;
    }

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

    MJMeshData* GetMeshData() {
        if (!mesh_storage_ || mesh_storage_->meshCount == 0) return nullptr;
        if (!mesh_data_) {
            mesh_data_ = std::make_unique<MJMeshData>(mesh_storage_.get());
        }
        return mesh_data_.get();
    }

private:
    // Per-actuator timeout policy: true = zero on timeout, false = hold last value
    void BuildActuatorTimeoutPolicy() {
        actuator_zero_on_timeout_.clear();
        if (!model_) return;
        actuator_zero_on_timeout_.resize(model_->nu);
        int zero_count = 0, hold_count = 0;
        for (int i = 0; i < model_->nu; i++) {
            switch (model_->actuator_biastype[i]) {
                case mjBIAS_AFFINE:
                    // Position/velocity servo → hold last value
                    actuator_zero_on_timeout_[i] = false;
                    hold_count++;
                    break;
                default:
                    // mjBIAS_NONE (torque), mjBIAS_MUSCLE, or unknown → zero on timeout
                    actuator_zero_on_timeout_[i] = true;
                    zero_count++;
                    break;
            }
        }
        os_log_info(OS_LOG_DEFAULT, "Actuator timeout policy: %d zero, %d hold, timeout=%ums",
                    zero_count, hold_count, ctrl_timeout_ms_);
    }

    void ExtractMeshData() {
        mesh_data_.reset();
        mesh_storage_.reset();

        if (!model_ || model_->nmesh == 0) return;

        auto storage = std::make_unique<MJMeshDataStorage>();
        storage->meshCount = model_->nmesh;
        storage->meshes.resize(model_->nmesh);

        // MuJoCo 3.4.0 uses separate indexing for positions (mesh_face) and
        // normals (mesh_facenormal). We unroll to per-face-vertex: each face
        // corner becomes a unique vertex with its own position + normal.
        // This duplicates positions but ensures correct per-face normals.

        // Count total unrolled vertices (3 per face)
        int totalFaces = 0;
        for (int m = 0; m < model_->nmesh; m++) {
            totalFaces += model_->mesh_facenum[m];
        }
        int totalUnrolledVerts = totalFaces * 3;

        storage->vertices.resize(totalUnrolledVerts * 6);  // 6 floats per vertex (pos[3] + normal[3])
        storage->faces.resize(totalUnrolledVerts);          // Sequential indices (0, 1, 2, ...)

        int vertOffset = 0;   // In unrolled vertices
        int faceOffset = 0;   // In unrolled faces (= vertices / 3)

        for (int m = 0; m < model_->nmesh; m++) {
            int nf = model_->mesh_facenum[m];
            int srcVertStart = model_->mesh_vertadr[m];
            int srcNormStart = model_->mesh_normaladr[m];
            int srcFaceStart = model_->mesh_faceadr[m];

            storage->meshes[m].vertexOffset = vertOffset;
            storage->meshes[m].vertexCount = nf * 3;
            storage->meshes[m].faceOffset = faceOffset;
            storage->meshes[m].faceCount = nf;

            // Unroll each face: look up position via mesh_face, normal via mesh_facenormal
            for (int f = 0; f < nf; f++) {
                for (int c = 0; c < 3; c++) {
                    int faceIdx = (srcFaceStart + f) * 3 + c;
                    int vi = model_->mesh_face[faceIdx];         // vertex index (relative to mesh)
                    int ni = model_->mesh_facenormal[faceIdx];   // normal index (relative to mesh)

                    int vSrc = (srcVertStart + vi) * 3;
                    int nSrc = (srcNormStart + ni) * 3;
                    int dstIdx = (vertOffset + f * 3 + c) * 6;

                    // Position
                    storage->vertices[dstIdx + 0] = model_->mesh_vert[vSrc + 0];
                    storage->vertices[dstIdx + 1] = model_->mesh_vert[vSrc + 1];
                    storage->vertices[dstIdx + 2] = model_->mesh_vert[vSrc + 2];
                    // Normal
                    storage->vertices[dstIdx + 3] = model_->mesh_normal[nSrc + 0];
                    storage->vertices[dstIdx + 4] = model_->mesh_normal[nSrc + 1];
                    storage->vertices[dstIdx + 5] = model_->mesh_normal[nSrc + 2];

                    // Sequential index (0-based per mesh)
                    storage->faces[faceOffset * 3 + f * 3 + c] = f * 3 + c;
                }
            }

            vertOffset += nf * 3;
            faceOffset += nf;
        }

        os_log_info(OS_LOG_DEFAULT, "Extracted mesh data: %d meshes, %d unrolled vertices, %d faces",
                    storage->meshCount, totalUnrolledVerts, totalFaces);

        mesh_storage_ = std::move(storage);
    }

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
        float current_sps_f = 0.0f;
        float current_tx_rate = 0.0f;
        float current_rx_rate = 0.0f;
        uint32_t prev_packets_sent = udp_server_.GetPacketsSent();
        uint32_t prev_packets_received = udp_server_.GetPacketsReceived();

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

            // === UDP Receive: buffer incoming controls ===
            if (udp_server_.IsActive()) {
                int packets_received = 0;
                while (packets_received < kMaxPacketsPerLoop) {
                    uint64_t host_ts = 0;
                    int received = udp_server_.ReceiveControl(ctrl_buffer.data(),
                                                              static_cast<int>(ctrl_buffer.size()),
                                                              &host_ts);
                    if (received < 0) break;

                    last_ctrl_received_ = Clock::now();
                    ctrl_timed_out_ = false;
                    packets_received++;

                    // Empty ctrl (received == 0) means "no change" — don't enqueue
                    if (received > 0) {
                        int nu = model_->nu;
                        int copy_count = std::min(received, nu);
                        TimestampedCtrl tc;
                        tc.host_timestamp_us = host_ts;
                        tc.ctrl.assign(ctrl_buffer.data(), ctrl_buffer.data() + copy_count);
                        ctrl_queue_.push_back(std::move(tc));
                    }
                }
            }

            // === Paced Replay: apply buffered controls at original cadence ===
            bool did_udp_step = false;
            if (!ctrl_queue_.empty()) {
                auto& next = ctrl_queue_.front();

                bool should_apply = false;
                if (next.host_timestamp_us == 0) {
                    // Legacy packet (no timestamp) — apply immediately
                    should_apply = true;
                } else {
                    if (replay_anchor_host_us_ == 0) {
                        // First timestamped packet in batch — set anchor
                        replay_anchor_cpu_ = Clock::now();
                        replay_anchor_host_us_ = next.host_timestamp_us;
                    }
                    if (next.host_timestamp_us <= replay_anchor_host_us_) {
                        // Out-of-order or backwards timestamp — apply immediately
                        should_apply = true;
                    } else {
                        uint64_t delta_us = next.host_timestamp_us - replay_anchor_host_us_;
                        auto elapsed = Clock::now() - replay_anchor_cpu_;
                        if (elapsed >= std::chrono::microseconds(delta_us)) {
                            should_apply = true;
                        }
                    }
                }

                if (should_apply) {
                    int nu = model_->nu;
                    int copy_count = std::min(static_cast<int>(next.ctrl.size()), nu);
                    for (int i = 0; i < copy_count; i++) {
                        data_->ctrl[i] = next.ctrl[i];
                    }
                    ctrl_queue_.pop_front();

                    // Reset anchor when queue drains
                    if (ctrl_queue_.empty()) {
                        replay_anchor_host_us_ = 0;
                    }

                    mj_step(model_, data_);
                    step_count++;
                    steps_since_last_frame++;
                    did_udp_step = true;

                    udp_server_.SendState(model_, data_);
                }
            }

            // === Ctrl Timeout: zero torque actuators if no packets received ===
            if (ctrl_timeout_ms_ > 0 && !ctrl_timed_out_
                && last_ctrl_received_.time_since_epoch().count() > 0) {
                auto since_last = Clock::now() - last_ctrl_received_;
                if (since_last >= std::chrono::milliseconds(ctrl_timeout_ms_)) {
                    ctrl_timed_out_ = true;
                    int nu = model_->nu;
                    for (int i = 0; i < nu && i < static_cast<int>(actuator_zero_on_timeout_.size()); i++) {
                        if (actuator_zero_on_timeout_[i]) {
                            data_->ctrl[i] = 0.0;
                        }
                    }
                    // Clear any stale queued controls
                    ctrl_queue_.clear();
                    replay_anchor_host_us_ = 0;
                    replay_anchor_cpu_ = Clock::time_point{};
                    os_log_info(OS_LOG_DEFAULT, "Ctrl timeout: zeroed actuators after %ums",
                                ctrl_timeout_ms_);
                }
            }

            // Note: no resync after UDP steps — they consume time from the
            // real-time budget so total step rate stays at the expected real-time
            // rate regardless of driver packet rate.

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
                    // Send state to connected client even when stepping in
                    // real-time mode (not only on UDP-driven steps). This lets
                    // clients that send empty ctrl still receive state.
                    if (udp_server_.HasClient()) {
                        udp_server_.SendState(model_, data_);
                    }
                }
            }

            // Update stats (every second)
            auto now = Clock::now();
            auto stats_elapsed = Seconds(now - last_stats_update).count();
            if (stats_elapsed >= 1.0) {
                current_sps = static_cast<int32_t>(step_count / stats_elapsed);
                current_sps_f = static_cast<float>(step_count / stats_elapsed);

                uint32_t cur_sent = udp_server_.GetPacketsSent();
                uint32_t cur_received = udp_server_.GetPacketsReceived();
                current_tx_rate = static_cast<float>((cur_sent - prev_packets_sent) / stats_elapsed);
                current_rx_rate = static_cast<float>((cur_received - prev_packets_received) / stats_elapsed);
                prev_packets_sent = cur_sent;
                prev_packets_received = cur_received;

                step_count = 0;
                last_stats_update = now;
            }

            // Adaptive frame writing
            auto frame_elapsed = Seconds(now - last_frame_write).count();
            if (steps_since_last_frame > 0 && frame_elapsed >= kFrameWriteInterval) {
                WriteFrameToBuffer(current_sps, current_sps_f, current_tx_rate, current_rx_rate);
                last_frame_write = now;
                steps_since_last_frame = 0;
            }
        }
    }

    void WriteFrameToBuffer(int32_t sps = 0, float sps_f = 0.0f, float tx_rate = 0.0f, float rx_rate = 0.0f) {
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
            dst.dataid = src.dataid;
            dst.emission = src.emission;
            dst.specular = src.specular;
            dst.shininess = src.shininess;
        }

        // Copy lights from mjvScene
        frame->lightCount = std::min(scene_.nlight, MJ_MAX_LIGHTS);
        for (int i = 0; i < frame->lightCount; i++) {
            const mjvLight& src = scene_.lights[i];
            MJLightInstance& dst = frame->lights[i];
            memcpy(dst.pos, src.pos, 3 * sizeof(float));
            memcpy(dst.dir, src.dir, 3 * sizeof(float));
            memcpy(dst.ambient, src.ambient, 3 * sizeof(float));
            memcpy(dst.diffuse, src.diffuse, 3 * sizeof(float));
            memcpy(dst.specular, src.specular, 3 * sizeof(float));
            memcpy(dst.attenuation, src.attenuation, 3 * sizeof(float));
            dst.cutoff = src.cutoff;
            dst.exponent = src.exponent;
            dst.headlight = src.headlight;
            dst.directional = (src.type == mjLIGHT_DIRECTIONAL) ? 1 : 0;
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
        frame->stepsPerSecondF = sps_f;
        frame->txRate = tx_rate;
        frame->rxRate = rx_rate;
        // Use get_item_count() for actual frame count (not affected by signal_exit())
        frame->frameNumber = ring_buffer_.get_item_count() + 1;

        ring_buffer_.end_write();
    }

    // Timestamped control entry for paced replay queue
    struct TimestampedCtrl {
        uint64_t host_timestamp_us = 0;  // 0 = apply immediately (legacy)
        std::vector<double> ctrl;
    };

    const uint64_t unique_id_;  // Monotonic ID for per-thread state keying (survives address reuse)
    int32_t instance_index_;
    double target_fps_;
    bool busy_wait_;
    uint16_t udp_port_;
    uint32_t ctrl_timeout_ms_;
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
    // Epoch counter: incremented on Stop() so stale WaitForFrame() callers
    // from a previous run detect the generation change and return nullptr.
    std::atomic<uint64_t> epoch_{0};

    // Ctrl timeout and paced replay state (physics thread only)
    std::vector<bool> actuator_zero_on_timeout_;   // per-actuator: true = zero on timeout
    Clock::time_point last_ctrl_received_{};       // last time a UDP ctrl packet arrived
    bool ctrl_timed_out_ = false;                  // true when timeout has fired
    std::deque<TimestampedCtrl> ctrl_queue_;        // buffered controls awaiting replay
    Clock::time_point replay_anchor_cpu_{};         // wall-clock anchor for paced replay
    uint64_t replay_anchor_host_us_ = 0;           // host timestamp anchor (0 = no anchor)

    // Pre-loaded mesh data (extracted from mjModel at load time)
    std::unique_ptr<MJMeshDataStorage> mesh_storage_;
    std::unique_ptr<MJMeshData> mesh_data_;
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

const MJLightInstance* MJFrameDataGetLights(const MJFrameData* frame) {
    if (!frame || !frame->storage_) return nullptr;
    return frame->storage_->lights;
}

// MARK: - MJMeshData Free Functions

const MJMeshInfo* MJMeshDataGetMeshes(const MJMeshData* data) {
    if (!data || !data->storage_ || data->storage_->meshes.empty()) return nullptr;
    return data->storage_->meshes.data();
}

const float* MJMeshDataGetVertices(const MJMeshData* data) {
    if (!data || !data->storage_ || data->storage_->vertices.empty()) return nullptr;
    return data->storage_->vertices.data();
}

const int32_t* MJMeshDataGetFaces(const MJMeshData* data) {
    if (!data || !data->storage_ || data->storage_->faces.empty()) return nullptr;
    return data->storage_->faces.data();
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

void MJSimulationRuntime::setTimestep(double v) { impl_->SetTimestep(v); }
void MJSimulationRuntime::setRealtimeFactor(double v) { impl_->SetRealtimeFactor(v); }
double MJSimulationRuntime::getRealtimeFactor() const { return impl_->GetRealtimeFactor(); }

const mjvScene* MJSimulationRuntime::getScene() const { return impl_->GetScene(); }
mjvCamera* MJSimulationRuntime::getCamera() { return impl_->GetCamera(); }
mjvOption* MJSimulationRuntime::getOption() { return impl_->GetOption(); }
const mjModel* MJSimulationRuntime::getModel() const { return impl_->GetModel(); }
const mjData* MJSimulationRuntime::getData() const { return impl_->GetData(); }
MJMeshData* MJSimulationRuntime::getMeshData() { return impl_->GetMeshData(); }
