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

// MARK: - Forward Declarations

struct MJGrpcServerImpl;

// MARK: - MJGrpcServer

/// gRPC server for low-frequency simulation control.
/// High-frequency control/state streaming remains on FlatBuffers over UDP.
///
/// Direct C++ calls are used for: Start, Pause, Reset, ResetToKeyframe, Step,
/// GetState, ListInstances, GetInstanceInfo.
///
/// Callbacks to Swift are used for: LoadModel (needs Bundle path resolution),
/// Unload (needs Swift-level cleanup of streamers/UI).
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

private:
    explicit MJGrpcServer(std::unique_ptr<MJGrpcServerImpl> impl);
    std::unique_ptr<MJGrpcServerImpl> impl_;
};

#endif /* mjc_grpc_server_h */
