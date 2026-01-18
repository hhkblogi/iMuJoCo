// MJRuntime.swift
// Swift wrapper for MJPhysicsRuntime C interface

import Foundation
import MJCPhysicsRuntime

// MARK: - Runtime State

enum MJRuntimeSimulationState: Int32 {
    case inactive = 0
    case loaded = 1
    case running = 2
    case paused = 3

    init(from cState: MJRuntimeState) {
        self = MJRuntimeSimulationState(rawValue: Int32(cState.rawValue)) ?? .inactive
    }
}

// MARK: - Runtime Statistics

struct MJRuntimeStatistics {
    let simulationTime: Double
    let measuredSlowdown: Double
    let timestep: Double
    let stepsPerSecond: Int32
    // Network stats
    let udpPort: UInt16
    let packetsReceived: UInt32
    let packetsSent: UInt32
    let hasClient: Bool

    init(from cStats: MJRuntimeStats) {
        self.simulationTime = cStats.simulationTime
        self.measuredSlowdown = cStats.measuredSlowdown
        self.timestep = cStats.timestep
        self.stepsPerSecond = cStats.stepsPerSecond
        self.udpPort = cStats.udpPort
        self.packetsReceived = cStats.packetsReceived
        self.packetsSent = cStats.packetsSent
        self.hasClient = cStats.hasClient
    }
}

// MARK: - Runtime Error

enum MJRuntimeError: Error, LocalizedError {
    case creationFailed
    case loadFailed(String)
    case invalidHandle

    var errorDescription: String? {
        switch self {
        case .creationFailed: return "Failed to create runtime instance"
        case .loadFailed(let message): return "Failed to load model: \(message)"
        case .invalidHandle: return "Invalid runtime handle"
        }
    }
}

// MARK: - MJRuntime

/// Swift wrapper for MuJoCo Physics Runtime
/// Provides thread-safe simulation with proper CPU-simulation time synchronization
final class MJRuntime: @unchecked Sendable {
    private let handle: MJRuntimeHandle

    // MARK: - Initialization

    init(instanceIndex: Int32, targetFPS: Double = 60.0, busyWait: Bool = false, udpPort: UInt16 = 0) throws {
        var config = MJRuntimeConfig(
            instanceIndex: instanceIndex,
            targetFPS: targetFPS,
            busyWait: busyWait,
            udpPort: udpPort  // 0 = use default (8888 + instanceIndex)
        )

        guard let h = mj_runtime_create(&config) else {
            throw MJRuntimeError.creationFailed
        }
        self.handle = h
    }

    deinit {
        mj_runtime_destroy(handle)
    }

    // MARK: - Model Loading

    /// Load a model from an XML file path
    func loadModel(fromFile path: String) throws {
        var errorBuffer = [CChar](repeating: 0, count: 1024)
        let success = mj_runtime_load_model(
            handle,
            path,
            &errorBuffer,
            Int32(errorBuffer.count)
        )

        if !success {
            let errorMessage = String(cString: errorBuffer)
            throw MJRuntimeError.loadFailed(errorMessage)
        }
    }

    /// Load a model from an XML string
    func loadModel(fromXML xml: String) throws {
        var errorBuffer = [CChar](repeating: 0, count: 1024)
        let success = mj_runtime_load_model_xml(
            handle,
            xml,
            &errorBuffer,
            Int32(errorBuffer.count)
        )

        if !success {
            let errorMessage = String(cString: errorBuffer)
            throw MJRuntimeError.loadFailed(errorMessage)
        }
    }

    /// Unload the current model
    func unload() {
        mj_runtime_unload(handle)
    }

    // MARK: - Simulation Control

    /// Start the physics simulation (runs on dedicated C++ thread)
    func start() {
        mj_runtime_start(handle)
    }

    /// Pause the physics simulation
    func pause() {
        mj_runtime_pause(handle)
    }

    /// Reset the simulation to initial state
    func reset() {
        mj_runtime_reset(handle)
    }

    /// Step the simulation manually (when paused)
    func step() {
        mj_runtime_step(handle)
    }

    // MARK: - State Access

    /// Get the current simulation state
    var state: MJRuntimeSimulationState {
        MJRuntimeSimulationState(from: mj_runtime_get_state(handle))
    }

    /// Get simulation statistics
    var stats: MJRuntimeStatistics {
        MJRuntimeStatistics(from: mj_runtime_get_stats(handle))
    }

    /// Current simulation time
    var simulationTime: Double {
        stats.simulationTime
    }

    /// Model timestep
    var timestep: Double {
        stats.timestep
    }

    // MARK: - Network Access

    /// UDP port this instance is listening on (0 if not active)
    var udpPort: UInt16 {
        stats.udpPort
    }

    /// Whether a network client is connected
    var hasClient: Bool {
        stats.hasClient
    }

    /// Number of control packets received
    var packetsReceived: UInt32 {
        stats.packetsReceived
    }

    /// Number of state packets sent
    var packetsSent: UInt32 {
        stats.packetsSent
    }

    // MARK: - Thread-Safe Scene Access

    /// Lock the simulation mutex for safe access to scene data
    func lock() {
        mj_runtime_lock(handle)
    }

    /// Unlock the simulation mutex
    func unlock() {
        mj_runtime_unlock(handle)
    }

    /// Update the visualization scene with current simulation state
    /// Call this while holding the lock
    func updateScene() {
        mj_runtime_update_scene(handle)
    }

    /// Get pointer to mjvScene for rendering (only valid while locked)
    var scenePointer: UnsafePointer<mjvScene>? {
        mj_runtime_get_scene(handle)
    }

    /// Get pointer to mjvCamera (only valid while locked)
    var cameraPointer: UnsafeMutablePointer<mjvCamera>? {
        mj_runtime_get_camera(handle)
    }

    /// Get pointer to mjvOption (only valid while locked)
    var optionPointer: UnsafeMutablePointer<mjvOption>? {
        mj_runtime_get_option(handle)
    }

    /// Execute a closure while holding the lock
    func withLock<T>(_ body: () throws -> T) rethrows -> T {
        lock()
        defer { unlock() }
        return try body()
    }

    // MARK: - Camera Control

    var cameraAzimuth: Double {
        get {
            withLock { cameraPointer?.pointee.azimuth ?? 90.0 }
        }
        set {
            mj_runtime_set_camera_azimuth(handle, newValue)
        }
    }

    var cameraElevation: Double {
        get {
            withLock { cameraPointer?.pointee.elevation ?? -15.0 }
        }
        set {
            mj_runtime_set_camera_elevation(handle, newValue)
        }
    }

    var cameraDistance: Double {
        get {
            withLock { cameraPointer?.pointee.distance ?? 3.0 }
        }
        set {
            mj_runtime_set_camera_distance(handle, newValue)
        }
    }

    func setCameraLookat(x: Double, y: Double, z: Double) {
        mj_runtime_set_camera_lookat(handle, x, y, z)
    }

    func resetCamera() {
        mj_runtime_reset_camera(handle)
    }

    // MARK: - Real-time Control

    var realtimeFactor: Double {
        get { mj_runtime_get_realtime_factor(handle) }
        set { mj_runtime_set_realtime_factor(handle, newValue) }
    }

    // MARK: - Ring Buffer API (Lock-Free Frame Access)

    /// Get the latest available frame without waiting (non-blocking)
    /// May return the same frame multiple times if physics is slower than render
    /// Returns nil if no frame is available yet
    var latestFrame: UnsafePointer<MJFrameData>? {
        mj_runtime_get_latest_frame(handle)
    }

    /// Wait for a new frame from the physics thread (blocking)
    /// Use this when you want to synchronize render with physics
    /// The stored lastFrameCount should be passed and will be updated
    func waitForFrame() -> UnsafePointer<MJFrameData>? {
        mj_runtime_wait_for_frame(handle)
    }

    /// Get the current frame count (for tracking new frames)
    var frameCount: UInt64 {
        mj_runtime_get_frame_count(handle)
    }
}
