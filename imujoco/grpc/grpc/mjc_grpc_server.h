// mjc_grpc_server.h
// gRPC server for simulation control.
// Exposes only the server class and callback typedefs — no gRPC includes leak into Swift.

#ifndef mjc_grpc_server_h
#define mjc_grpc_server_h

#include <cstdint>
#include <memory>

// Swift C++ interop: for non-copyable types with reference semantics
#ifndef SWIFT_IMMORTAL_REFERENCE
#if __has_attribute(swift_attr)
#define SWIFT_IMMORTAL_REFERENCE __attribute__((swift_attr("import_reference"))) __attribute__((swift_attr("retain:immortal"))) __attribute__((swift_attr("release:immortal")))
#else
#define SWIFT_IMMORTAL_REFERENCE
#endif
#endif

// Forward declarations (no #include of physics header)
class MJSimulationRuntime;

// MARK: - gRPC Operation Enum

/// Operations dispatched via the operation callback.
enum MJGrpcOp : int32_t {
    MJ_GRPC_OP_START = 0,
    MJ_GRPC_OP_PAUSE = 1,
    MJ_GRPC_OP_RESET = 2,
    MJ_GRPC_OP_STEP = 3,
    MJ_GRPC_OP_RESET_TO_KEYFRAME = 4,
};

// MARK: - Physics State Struct (for GetState callback)

static constexpr int32_t kMJGrpcMaxDof = 500;
static constexpr int32_t kMJGrpcMaxSensor = 2048;

/// Fixed-size struct filled by the GetState callback on MainActor.
/// Allocated on stack by the gRPC handler, filled by the callback.
struct MJGrpcPhysicsState {
    bool valid = false;
    double time = 0.0;
    int32_t nq = 0, nv = 0, nu = 0, nsensordata = 0;
    double qpos[kMJGrpcMaxDof] = {};
    double qvel[kMJGrpcMaxDof] = {};
    double ctrl[kMJGrpcMaxDof] = {};
    double sensordata[kMJGrpcMaxSensor] = {};
    double energyPotential = 0.0;
    double energyKinetic = 0.0;
};

/// Fill a MJGrpcPhysicsState from the runtime's current mjModel/mjData.
/// Must be called while the runtime is safe to access (e.g. on MainActor).
void MJGrpcFillPhysicsState(MJGrpcPhysicsState* outState, MJSimulationRuntime* runtime);

// MARK: - Configuration

struct MJGrpcServerConfig {
    uint16_t port = 8999;
    int32_t numInstances = 4;
};

// MARK: - Callback Typedefs

/// Called when a gRPC client requests model loading.
/// @param instanceId 1-based instance ID
/// @param modelName Model name from the asset catalogue
/// @param context Opaque pointer passed to setLoadCallback
/// @return true on success
typedef bool (*MJGrpcLoadModelCallback)(int32_t instanceId, const char* modelName, void* context);

/// Called when a gRPC client requests unloading.
/// @param instanceId 1-based instance ID
/// @param context Opaque pointer passed to setUnloadCallback
/// @return true on success
typedef bool (*MJGrpcUnloadCallback)(int32_t instanceId, void* context);

/// Called for Start/Pause/Reset/Step/ResetToKeyframe RPCs.
/// @param instanceId 1-based instance ID
/// @param op MJGrpcOp enum value
/// @param param Extra parameter (keyframe index for RESET_TO_KEYFRAME, 0 otherwise)
/// @param context Opaque pointer passed to setOperationCallback
/// @return true on success
typedef bool (*MJGrpcOperationCallback)(int32_t instanceId, int32_t op, int32_t param, void* context);

/// Called for GetState RPCs. Fills outState on the MainActor thread.
/// @param instanceId 1-based instance ID
/// @param outState Caller-allocated struct to fill with physics state
/// @param context Opaque pointer passed to setGetStateCallback
/// @return true on success
typedef bool (*MJGrpcGetStateCallback)(int32_t instanceId, MJGrpcPhysicsState* outState, void* context);

// MARK: - Forward Declarations

struct MJGrpcServerImpl;

// MARK: - MJGrpcServer

/// gRPC server for low-frequency simulation control.
/// High-frequency control/state streaming remains on FlatBuffers over UDP.
///
/// All mutating RPCs are routed through callbacks that execute on MainActor
/// to avoid races with UI-initiated mutations on the same simulation instance.
///
/// Callbacks to Swift (via MainActor): LoadModel, Unload, Start, Pause, Reset,
/// ResetToKeyframe, Step, GetState.
///
/// Direct access (safe under gRPC mutex): ListInstances, GetInstanceInfo.
class SWIFT_IMMORTAL_REFERENCE MJGrpcServer {
public:
    /// Create a new gRPC server
    /// @param config Server configuration
    /// @return Pointer to server (caller owns), or nullptr on failure
    static MJGrpcServer* create(const MJGrpcServerConfig& config);

    /// Destroy a gRPC server (call when done)
    static void destroy(MJGrpcServer* server);

    ~MJGrpcServer();

    // Non-copyable
    MJGrpcServer(const MJGrpcServer&) = delete;
    MJGrpcServer& operator=(const MJGrpcServer&) = delete;

    // MARK: - Server Lifecycle

    /// Start listening for gRPC connections
    void start();

    /// Shutdown the gRPC server
    void stop();

    /// Number of RPCs processed since server start (monotonically increasing).
    uint64_t rpcCount() const;

    // MARK: - Runtime Registration

    /// Register a C++ physics runtime for direct gRPC access.
    /// @param instanceId 1-based instance ID
    /// @param runtime Pointer to the MJSimulationRuntime (not retained)
    /// @param modelName Display name for the loaded model
    void registerRuntime(int32_t instanceId, MJSimulationRuntime* runtime, const char* modelName);

    /// Unregister a runtime (call before destroying it).
    /// @param instanceId 1-based instance ID
    void unregisterRuntime(int32_t instanceId);

    // MARK: - Callbacks

    /// Set the callback for gRPC LoadModel requests.
    void setLoadCallback(MJGrpcLoadModelCallback callback, void* context);

    /// Set the callback for gRPC Unload requests.
    void setUnloadCallback(MJGrpcUnloadCallback callback, void* context);

    /// Set the callback for gRPC operation requests (Start/Pause/Reset/Step/ResetToKeyframe).
    void setOperationCallback(MJGrpcOperationCallback callback, void* context);

    /// Set the callback for gRPC GetState requests.
    void setGetStateCallback(MJGrpcGetStateCallback callback, void* context);

private:
    explicit MJGrpcServer(std::unique_ptr<MJGrpcServerImpl> impl);
    std::unique_ptr<MJGrpcServerImpl> impl_;
};

#endif /* mjc_grpc_server_h */
