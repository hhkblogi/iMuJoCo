// mjc_physics_runtime.h
// C++ interface for MuJoCo physics simulation runtime
// Uses mutex-free ring buffer with C++20 atomic wait/notify for lowest latency

#ifndef mjc_physics_runtime_h
#define mjc_physics_runtime_h

#include <cstdint>
#include <memory>
#include <string>

// Swift C++ interop: for non-copyable types with reference semantics
#if __has_attribute(swift_attr)
#define SWIFT_IMMORTAL_REFERENCE __attribute__((swift_attr("import_reference"))) __attribute__((swift_attr("retain:immortal"))) __attribute__((swift_attr("release:immortal")))
#else
#define SWIFT_IMMORTAL_REFERENCE
#endif

// Forward declare MuJoCo types to avoid including mujoco.h in Swift context
// These are opaque pointers for Swift; full access requires C++ code
struct mjModel_;
typedef struct mjModel_ mjModel;
struct mjData_;
typedef struct mjData_ mjData;
struct mjvScene_;
typedef struct mjvScene_ mjvScene;
struct mjvCamera_;
typedef struct mjvCamera_ mjvCamera;
struct mjvOption_;
typedef struct mjvOption_ mjvOption;

// MARK: - Constants

constexpr int MJ_MAX_GEOMS = 10000;
constexpr uint16_t MJ_DEFAULT_UDP_PORT = 8888;

// UDP Packet magic number (see mjc_fragment.h for fragment protocol)
constexpr uint32_t MJ_PACKET_MAGIC_FRAG = 0x4D4A4647;  // "MJFG" - Fragment packet

// MARK: - Types

/// Simulation state enum
enum class MJRuntimeState : int32_t {
    Inactive = 0,
    Loaded = 1,
    Running = 2,
    Paused = 3
};

/// Configuration for creating a runtime instance
struct MJRuntimeConfig {
    int32_t instanceIndex = 0;      ///< Instance ID (0-3)
    double targetFPS = 60.0;        ///< Target simulation FPS
    bool busyWait = false;          ///< Use busy wait for better timing (uses more CPU)
    uint16_t udpPort = 0;           ///< UDP port for network I/O (0 = default: 8888 + instanceIndex)
};

/// Statistics from the physics runtime
struct MJRuntimeStats {
    double simulationTime = 0.0;        ///< Current simulation time
    double measuredSlowdown = 1.0;      ///< Measured slowdown factor
    double timestep = 0.002;            ///< Model timestep
    int32_t stepsPerSecond = 0;         ///< Measured physics steps per second

    // Network stats
    uint16_t udpPort = 0;               ///< UDP port being used (0 if disabled)
    uint32_t packetsReceived = 0;       ///< Total control packets received
    uint32_t packetsSent = 0;           ///< Total state packets sent
    bool hasClient = false;             ///< True if a client is connected
};

// MARK: - GeomInstance (for instanced rendering)

struct MJGeomInstance {
    // Transform
    float pos[3] = {0, 0, 0};               ///< World position
    float mat[9] = {1,0,0, 0,1,0, 0,0,1};   ///< 3x3 rotation matrix
    float size[3] = {1, 1, 1};              ///< Geom size parameters

    // Visual properties
    float rgba[4] = {1, 1, 1, 1};           ///< Color
    int32_t type = 0;                       ///< Geom type (sphere, capsule, box, etc.)
    float emission = 0.0f;                  ///< Emission
    float specular = 0.5f;                  ///< Specular
    float shininess = 0.5f;                 ///< Shininess
};

// MARK: - FrameDataStorage (internal ring buffer storage - not exposed to Swift)
// This struct is stored by value in the ring buffer. It's large (~920KB) due to
// the embedded geom array. Swift should never access this type directly.

struct MJFrameDataStorage {
    MJGeomInstance geoms[MJ_MAX_GEOMS];
    int32_t geomCount = 0;

    // Camera state
    float cameraPos[3] = {0, 0, 0};
    float cameraLookat[3] = {0, 0, 0};
    float cameraAzimuth = 0.0f;
    float cameraElevation = 0.0f;
    float cameraDistance = 3.0f;

    // Timing
    double simulationTime = 0.0;
    int32_t stepsPerSecond = 0;

    // Frame sequence number (for debugging)
    uint64_t frameNumber = 0;
};

// MARK: - MJFrameData (Swift-facing view class with reference semantics)
// This is a lightweight wrapper (~8 bytes) that provides safe access to frame data.
// Uses SWIFT_IMMORTAL_REFERENCE so Swift treats it as a reference type, avoiding
// any copying of the underlying large storage.
//
// IMPORTANT: Only the most recent pointer returned by getLatestFrame() or waitForFrame()
// on a given thread is guaranteed valid. It becomes invalid on the next call to either
// function on that thread. Do not cache or store these pointers across function calls.
//
// The underlying ring buffer uses a fixed-size N-slot array. Storage slots are never
// deallocated during the runtime's lifetime. However, the storage contents may be
// overwritten by the physics thread after N-1 subsequent writes. Callers should copy
// frame data promptly within a single render pass.

class SWIFT_IMMORTAL_REFERENCE MJFrameData {
public:
    /// Create a view wrapping the given storage
    explicit MJFrameData(const MJFrameDataStorage* storage) : storage_(storage) {}

    // Non-copyable to enforce reference semantics
    MJFrameData(const MJFrameData&) = delete;
    MJFrameData& operator=(const MJFrameData&) = delete;

    // Friend function for geom access (member functions returning pointers not supported in Swift)
    friend const MJGeomInstance* MJFrameDataGetGeoms(const MJFrameData* frame);

    // MARK: - Accessors

    /// Number of geometry instances in this frame
    int32_t geomCount() const { return storage_ ? storage_->geomCount : 0; }

    /// Camera azimuth angle in degrees
    float cameraAzimuth() const { return storage_ ? storage_->cameraAzimuth : 0.0f; }

    /// Camera elevation angle in degrees
    float cameraElevation() const { return storage_ ? storage_->cameraElevation : 0.0f; }

    /// Camera distance from lookat point
    float cameraDistance() const { return storage_ ? storage_->cameraDistance : 3.0f; }

    /// Camera lookat X coordinate
    float cameraLookatX() const { return storage_ ? storage_->cameraLookat[0] : 0.0f; }

    /// Camera lookat Y coordinate
    float cameraLookatY() const { return storage_ ? storage_->cameraLookat[1] : 0.0f; }

    /// Camera lookat Z coordinate
    float cameraLookatZ() const { return storage_ ? storage_->cameraLookat[2] : 0.0f; }

    /// Current simulation time
    double simulationTime() const { return storage_ ? storage_->simulationTime : 0.0; }

    /// Frame sequence number
    uint64_t frameNumber() const { return storage_ ? storage_->frameNumber : 0; }

private:
    const MJFrameDataStorage* storage_;
};

// MARK: - MJFrameData Free Functions (for Swift access)
// Swift C++ interop doesn't support member functions that return pointers,
// so we provide these free functions instead.

/// Get pointer to the contiguous geoms array for indexed access.
///
/// @warning No bounds checking is performed. Accessing indices outside
/// `[0, frame->geomCount() - 1]` results in undefined behavior.
///
/// The returned array contains exactly `frame->geomCount()` elements.
/// The pointer is owned by `frame` and is valid as long as the underlying
/// MJFrameDataStorage outlives all accesses.
///
/// @param frame The frame data view
/// @return Pointer to first geom, or nullptr if frame is null
const MJGeomInstance* MJFrameDataGetGeoms(const MJFrameData* frame);

// MARK: - Version Info

/// Get MuJoCo version number (e.g., 340 for version 3.4.0)
int32_t MJGetVersion();

// MARK: - Forward Declarations

class MJSimulationRuntimeImpl;

// MARK: - MJSimulationRuntime

/// MuJoCo physics simulation runtime.
///
/// This class manages a MuJoCo simulation running on a dedicated physics thread.
/// It uses a mutex-free ring buffer for thread-safe frame access from the render thread.
///
/// ## Thread Safety
///
/// **Thread-safe (can be called from any thread):**
/// - `getLatestFrame()`, `waitForFrame()`, `getFrameCount()` - mutex-free ring buffer
/// - `getState()`, `getStats()` - atomic reads
///
/// **Single-thread only (call from owner thread):**
/// - `start()`, `pause()`, `reset()`, `step()` - control methods
/// - `loadModel()`, `loadModelXML()`, `unload()` - model management
/// - Camera setters
///
/// ## Usage Example
///
/// ```cpp
/// auto runtime = MJSimulationRuntime::create({.instanceIndex = 0});
/// if (runtime->loadModel("humanoid.xml")) {
///     runtime->start();
///     while (running) {
///         const auto* frame = runtime->getLatestFrame();
///         // Render frame...
///     }
///     runtime->pause();
/// }
/// ```
class SWIFT_IMMORTAL_REFERENCE MJSimulationRuntime {
public:
    /// Create a new runtime instance
    /// @param config Configuration options
    /// @return Pointer to runtime (caller owns), or nullptr on failure
    static MJSimulationRuntime* create(const MJRuntimeConfig& config);

    /// Destroy a runtime instance (call when done)
    /// @param runtime Pointer returned by create()
    static void destroy(MJSimulationRuntime* runtime);

    /// Destructor - stops simulation and frees resources
    ~MJSimulationRuntime();

    // Non-copyable, non-movable
    MJSimulationRuntime(const MJSimulationRuntime&) = delete;
    MJSimulationRuntime& operator=(const MJSimulationRuntime&) = delete;
    MJSimulationRuntime(MJSimulationRuntime&&) = delete;
    MJSimulationRuntime& operator=(MJSimulationRuntime&&) = delete;

    // MARK: - Model Loading

    /// Load a model from an XML file path
    /// @param xmlPath Path to the XML file
    /// @param outError Optional pointer to receive error message
    /// @return true on success
    bool loadModel(const char* xmlPath, std::string* outError = nullptr);

    /// Load a model from an XML string
    /// @param xmlString XML content
    /// @param outError Optional pointer to receive error message
    /// @return true on success
    bool loadModelXML(const char* xmlString, std::string* outError = nullptr);

    /// Unload the current model
    void unload();

    // MARK: - Simulation Control

    /// Start the physics simulation (runs on dedicated thread)
    void start();

    /// Pause the physics simulation
    void pause();

    /// Reset the simulation to initial state
    /// @note Call pause() first for thread safety
    void reset();

    /// Step the simulation manually (when paused)
    void step();

    /// Get the current simulation state
    MJRuntimeState getState() const;

    /// Get simulation statistics
    MJRuntimeStats getStats() const;

    // MARK: - Frame Access
    // Thread-safe: Each thread gets its own view object via thread-local storage.
    // The underlying ring buffer storage is mutex-free for producer/consumer access.

    /// Get the latest available frame (non-blocking, thread-safe).
    /// @return Pointer to frame view, or nullptr if no frames have been written
    ///         yet (e.g., before the first simulation step). Valid until the next
    ///         getLatestFrame() or waitForFrame() call on the SAME THREAD. The
    ///         underlying storage remains valid as long as the runtime exists.
    MJFrameData* getLatestFrame();

    /// Wait for a new frame (blocking, thread-safe).
    /// @return Pointer to frame view, or nullptr if shutdown signaled.
    ///         Same lifetime semantics as getLatestFrame().
    MJFrameData* waitForFrame();

    /// Get the current frame count
    uint64_t getFrameCount() const;

    // MARK: - Camera Control

    void setCameraAzimuth(double azimuth);
    void setCameraElevation(double elevation);
    void setCameraDistance(double distance);
    void setCameraLookat(double x, double y, double z);
    void resetCamera();

    double getCameraAzimuth() const;
    double getCameraElevation() const;
    double getCameraDistance() const;

    // MARK: - Real-time Control

    void setRealtimeFactor(double factor);
    double getRealtimeFactor() const;

    // MARK: - Legacy Access (for compatibility with C++ code that needs MuJoCo types)
    // Note: These return opaque pointers when used from Swift

    /// Get pointer to mjvScene (prefer getLatestFrame for thread safety)
    const mjvScene* getScene() const;

    /// Get pointer to mjvCamera
    mjvCamera* getCamera();

    /// Get pointer to mjvOption
    mjvOption* getOption();

    /// Get pointer to mjModel (read-only)
    const mjModel* getModel() const;

    /// Get pointer to mjData (read-only)
    const mjData* getData() const;

private:
    /// Private constructor - use create()
    explicit MJSimulationRuntime(std::unique_ptr<MJSimulationRuntimeImpl> impl);

    std::unique_ptr<MJSimulationRuntimeImpl> impl_;
};

#endif /* mjc_physics_runtime_h */
