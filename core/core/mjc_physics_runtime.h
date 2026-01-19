// mjc_physics_runtime.h
// C interface for MuJoCo physics simulation runtime
// Uses lock-free ring buffer with C++20 atomic wait/notify for lowest latency

#ifndef mjc_physics_runtime_h
#define mjc_physics_runtime_h

#include <mujoco/mujoco.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// MARK: - Constants

#define MJ_MAX_GEOMS 10000
#define MJ_DEFAULT_UDP_PORT 8888

// UDP Packet magic numbers
#define MJ_PACKET_MAGIC_CTRL  0x4D4A4354  // "MJCT" - Control packet
#define MJ_PACKET_MAGIC_STATE 0x4D4A5354  // "MJST" - State packet

// MARK: - Types

// Opaque handle to a simulation instance
typedef void* MJRuntimeHandle;

// Simulation state enum
typedef enum {
    MJRuntimeStateInactive = 0,
    MJRuntimeStateLoaded,
    MJRuntimeStateRunning,
    MJRuntimeStatePaused
} MJRuntimeState;

// Configuration for creating a runtime instance
typedef struct {
    int32_t instanceIndex;          // Instance ID (0-3)
    double targetFPS;               // Target simulation FPS (default: 60)
    bool busyWait;                  // Use busy wait for better timing (uses more CPU)
    uint16_t udpPort;               // UDP port for network I/O (0 = disabled, default: 8888 + instanceIndex)
} MJRuntimeConfig;

// Statistics from the physics runtime
typedef struct {
    double simulationTime;          // Current simulation time
    double measuredSlowdown;        // Measured slowdown factor
    double timestep;                // Model timestep
    int32_t stepsPerSecond;         // Measured physics steps per second
    // Network stats
    uint16_t udpPort;               // UDP port being used (0 if disabled)
    uint32_t packetsReceived;       // Total control packets received
    uint32_t packetsSent;           // Total state packets sent
    bool hasClient;                 // True if a client is connected
} MJRuntimeStats;

// MARK: - GeomInstance (for instanced rendering)

typedef struct {
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
} MJGeomInstance;

// MARK: - FrameData (complete frame for rendering from ring buffer)

typedef struct {
    // Geom instances for rendering
    MJGeomInstance geoms[MJ_MAX_GEOMS];
    int32_t geom_count;

    // Camera state
    float camera_pos[3];
    float camera_lookat[3];
    float camera_azimuth;
    float camera_elevation;
    float camera_distance;

    // Timing
    double simulation_time;
    int32_t steps_per_second;

    // Frame sequence number (for debugging)
    uint64_t frame_number;
} MJFrameData;

// MARK: - Lifecycle

/// Create a new runtime instance with the given configuration
/// Returns NULL on failure
MJRuntimeHandle mjc_runtime_create(const MJRuntimeConfig* config);

/// Destroy a runtime instance and free all resources
void mjc_runtime_destroy(MJRuntimeHandle handle);

// MARK: - Model Loading

/// Load a model from an XML file path
bool mjc_runtime_load_model(MJRuntimeHandle handle,
                           const char* xmlPath,
                           char* errorBuffer,
                           int32_t errorBufferSize);

/// Load a model from an XML string
bool mjc_runtime_load_model_xml(MJRuntimeHandle handle,
                               const char* xmlString,
                               char* errorBuffer,
                               int32_t errorBufferSize);

/// Unload the current model
void mjc_runtime_unload(MJRuntimeHandle handle);

// MARK: - Simulation Control

/// Start the physics simulation (runs on dedicated thread)
void mjc_runtime_start(MJRuntimeHandle handle);

/// Pause the physics simulation
void mjc_runtime_pause(MJRuntimeHandle handle);

/// Reset the simulation to initial state.
/// WARNING: Not thread-safe while simulation is running.
/// Call mjc_runtime_pause() first, then reset, then mjc_runtime_start().
void mjc_runtime_reset(MJRuntimeHandle handle);

/// Step the simulation manually (when paused)
void mjc_runtime_step(MJRuntimeHandle handle);

/// Get the current simulation state
MJRuntimeState mjc_runtime_get_state(MJRuntimeHandle handle);

/// Get simulation statistics
MJRuntimeStats mjc_runtime_get_stats(MJRuntimeHandle handle);

// MARK: - Ring Buffer API (Lock-Free Frame Access)

/// Wait for a new frame from the physics thread (blocking)
/// Returns pointer to the latest completed frame
/// Use this when you want to synchronize render with physics
const MJFrameData* mjc_runtime_wait_for_frame(MJRuntimeHandle handle);

/// Get the latest available frame without waiting (non-blocking)
/// May return the same frame multiple times if physics is slower than render
/// Use this for non-blocking render loop
const MJFrameData* mjc_runtime_get_latest_frame(MJRuntimeHandle handle);

/// Get the current frame count (for tracking new frames)
uint64_t mjc_runtime_get_frame_count(MJRuntimeHandle handle);

// MARK: - Frame Data Accessors (for Swift interop)
// These are static inline so they can be used without linking to core

/// Get pointer to the geoms array in a frame
static inline const MJGeomInstance* mjc_frame_get_geoms(const MJFrameData* frame) {
    return frame ? frame->geoms : NULL;
}

/// Get the geom count from a frame
static inline int32_t mjc_frame_get_geom_count(const MJFrameData* frame) {
    return frame ? frame->geom_count : 0;
}

/// Get a specific geom by index from a frame
static inline const MJGeomInstance* mjc_frame_get_geom(const MJFrameData* frame, int32_t index) {
    if (!frame || index < 0 || index >= frame->geom_count) {
        return NULL;
    }
    return &frame->geoms[index];
}

// MARK: - Legacy Scene Access (for compatibility)

/// Lock/unlock are no-ops with ring buffer, kept for API compatibility
void mjc_runtime_lock(MJRuntimeHandle handle);
void mjc_runtime_unlock(MJRuntimeHandle handle);

/// Update scene - no-op with ring buffer, scene is updated automatically
void mjc_runtime_update_scene(MJRuntimeHandle handle);

/// Get pointer to mjvScene (legacy, prefer ring buffer API)
const mjvScene* mjc_runtime_get_scene(MJRuntimeHandle handle);

/// Get pointer to mjvCamera
mjvCamera* mjc_runtime_get_camera(MJRuntimeHandle handle);

/// Get pointer to mjvOption
mjvOption* mjc_runtime_get_option(MJRuntimeHandle handle);

/// Get pointer to mjModel (read-only)
const mjModel* mjc_runtime_get_model(MJRuntimeHandle handle);

/// Get pointer to mjData (read-only)
const mjData* mjc_runtime_get_data(MJRuntimeHandle handle);

// MARK: - Camera Control
// NOTE: Camera methods are not synchronized with physics thread.
// For best results, call these when simulation is paused or from main thread only.

void mjc_runtime_set_camera_azimuth(MJRuntimeHandle handle, double azimuth);
void mjc_runtime_set_camera_elevation(MJRuntimeHandle handle, double elevation);
void mjc_runtime_set_camera_distance(MJRuntimeHandle handle, double distance);
void mjc_runtime_set_camera_lookat(MJRuntimeHandle handle, double x, double y, double z);
void mjc_runtime_reset_camera(MJRuntimeHandle handle);

// MARK: - Real-time Control

void mjc_runtime_set_realtime_factor(MJRuntimeHandle handle, double factor);
double mjc_runtime_get_realtime_factor(MJRuntimeHandle handle);

#ifdef __cplusplus
}
#endif

#endif /* mjc_physics_runtime_h */
