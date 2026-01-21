// mjc_runtime.swift
// Swift wrapper for MJCPhysicsRuntime C interface

import Foundation
import MJCPhysicsRuntime

// MARK: - Runtime State

public enum MJRuntimeSimulationState: Int32 {
    case inactive = 0
    case loaded = 1
    case running = 2
    case paused = 3

    public init(from cState: MJRuntimeState) {
        self = MJRuntimeSimulationState(rawValue: Int32(cState.rawValue)) ?? .inactive
    }
}

// MARK: - Runtime Statistics

public struct MJRuntimeStatistics {
    public let simulationTime: Double
    public let measuredSlowdown: Double
    public let timestep: Double
    public let stepsPerSecond: Int32
    // Network stats
    public let udpPort: UInt16
    public let packetsReceived: UInt32
    public let packetsSent: UInt32
    public let hasClient: Bool

    public init(from cStats: MJRuntimeStats) {
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

public enum MJRuntimeError: Error, LocalizedError {
    case creationFailed
    case loadFailed(String)
    case invalidHandle

    public var errorDescription: String? {
        switch self {
        case .creationFailed: return "Failed to create runtime instance"
        case .loadFailed(let message): return "Failed to load model: \(message)"
        case .invalidHandle: return "Invalid runtime handle"
        }
    }
}

// MARK: - MJRuntime

/// Swift wrapper for MuJoCo Physics Runtime.
/// Provides simulation with proper CPU-simulation time synchronization.
///
/// ## Thread Safety
///
/// **Thread-safe (can be called from any thread):**
/// - `latestFrame`, `waitForFrame`, `frameCount` - lock-free ring buffer access
///
/// **Single-thread only (call from main thread or owner thread):**
/// - `start()`, `pause()`, `reset()`, `step()` - control methods
/// - Camera setters (`cameraAzimuth`, `cameraElevation`, etc.)
/// - `loadModel()`, `unload()`
///
/// **Safe reset sequence:** `pause()` → `reset()` → `start()`
public final class MJRuntime {
    private let handle: MJRuntimeHandle

    // MARK: - Initialization

    public init(instanceIndex: Int32, targetFPS: Double = 60.0, busyWait: Bool = false, udpPort: UInt16 = 0) throws {
        var config = MJRuntimeConfig(
            instanceIndex: instanceIndex,
            targetFPS: targetFPS,
            busyWait: busyWait,
            udpPort: udpPort  // 0 = use default (8888 + instanceIndex)
        )

        guard let h = mjc_runtime_create(&config) else {
            throw MJRuntimeError.creationFailed
        }
        self.handle = h
    }

    deinit {
        mjc_runtime_destroy(handle)
    }

    // MARK: - Model Loading

    /// Load a model from an XML file path
    public func loadModel(fromFile path: String) throws {
        var errorBuffer = [CChar](repeating: 0, count: 1024)
        let success = mjc_runtime_load_model(
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
    public func loadModel(fromXML xml: String) throws {
        var errorBuffer = [CChar](repeating: 0, count: 1024)
        let success = mjc_runtime_load_model_xml(
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
    public func unload() {
        mjc_runtime_unload(handle)
    }

    // MARK: - Simulation Control

    /// Start the physics simulation (runs on dedicated C++ thread)
    public func start() {
        mjc_runtime_start(handle)
    }

    /// Pause the physics simulation
    public func pause() {
        mjc_runtime_pause(handle)
    }

    /// Reset the simulation to initial state
    public func reset() {
        mjc_runtime_reset(handle)
    }

    /// Step the simulation manually (when paused)
    public func step() {
        mjc_runtime_step(handle)
    }

    // MARK: - State Access

    /// Get the current simulation state
    public var state: MJRuntimeSimulationState {
        MJRuntimeSimulationState(from: mjc_runtime_get_state(handle))
    }

    /// Get simulation statistics
    public var stats: MJRuntimeStatistics {
        MJRuntimeStatistics(from: mjc_runtime_get_stats(handle))
    }

    /// Current simulation time
    public var simulationTime: Double {
        stats.simulationTime
    }

    /// Model timestep
    public var timestep: Double {
        stats.timestep
    }

    // MARK: - Network Access

    /// UDP port this instance is listening on (0 if not active)
    public var udpPort: UInt16 {
        stats.udpPort
    }

    /// Whether a network client is connected
    public var hasClient: Bool {
        stats.hasClient
    }

    /// Number of control packets received
    public var packetsReceived: UInt32 {
        stats.packetsReceived
    }

    /// Number of state packets sent
    public var packetsSent: UInt32 {
        stats.packetsSent
    }

    // MARK: - Scene Access
    // NOTE: With lock-free ring buffer, these methods are for compatibility.
    // Prefer using latestFrame/waitForFrame for thread-safe frame access.

    /// Lock placeholder (no-op with current lock-free implementation).
    /// Retained for API compatibility; does not provide synchronization.
    public func lock() {
        mjc_runtime_lock(handle)
    }

    /// Unlock placeholder (no-op with current lock-free implementation).
    public func unlock() {
        mjc_runtime_unlock(handle)
    }

    /// Update the visualization scene with current simulation state.
    public func updateScene() {
        mjc_runtime_update_scene(handle)
    }

    /// Get pointer to mjvScene for rendering.
    /// WARNING: Not thread-safe while physics is running. Use latestFrame instead.
    public var scenePointer: UnsafePointer<mjvScene>? {
        mjc_runtime_get_scene(handle)
    }

    /// Get pointer to mjvCamera.
    /// WARNING: Not thread-safe while physics is running.
    public var cameraPointer: UnsafeMutablePointer<mjvCamera>? {
        mjc_runtime_get_camera(handle)
    }

    /// Get pointer to mjvOption.
    /// WARNING: Not thread-safe while physics is running.
    public var optionPointer: UnsafeMutablePointer<mjvOption>? {
        mjc_runtime_get_option(handle)
    }

    /// Execute a closure (lock/unlock are no-ops with lock-free implementation).
    /// NOTE: This does NOT provide thread synchronization. Use for API compatibility only.
    public func withLock<T>(_ body: () throws -> T) rethrows -> T {
        lock()
        defer { unlock() }
        return try body()
    }

    // MARK: - Camera Control

    public var cameraAzimuth: Double {
        get {
            withLock { cameraPointer?.pointee.azimuth ?? 90.0 }
        }
        set {
            mjc_runtime_set_camera_azimuth(handle, newValue)
        }
    }

    public var cameraElevation: Double {
        get {
            withLock { cameraPointer?.pointee.elevation ?? -15.0 }
        }
        set {
            mjc_runtime_set_camera_elevation(handle, newValue)
        }
    }

    public var cameraDistance: Double {
        get {
            withLock { cameraPointer?.pointee.distance ?? 3.0 }
        }
        set {
            mjc_runtime_set_camera_distance(handle, newValue)
        }
    }

    public func setCameraLookat(x: Double, y: Double, z: Double) {
        mjc_runtime_set_camera_lookat(handle, x, y, z)
    }

    public func resetCamera() {
        mjc_runtime_reset_camera(handle)
    }

    // MARK: - Real-time Control

    public var realtimeFactor: Double {
        get { mjc_runtime_get_realtime_factor(handle) }
        set { mjc_runtime_set_realtime_factor(handle, newValue) }
    }

    // MARK: - Ring Buffer API (Lock-Free Frame Access)

    /// Get the latest available frame without waiting (non-blocking)
    /// May return the same frame multiple times if physics is slower than render
    /// Returns nil if no frame is available yet
    public var latestFrame: UnsafePointer<MJFrameData>? {
        mjc_runtime_get_latest_frame(handle)
    }

    /// Wait for a new frame from the physics thread (blocking).
    /// Use this when you want to synchronize rendering with physics.
    /// The runtime internally tracks the last delivered frame, blocking
    /// until a strictly newer frame is available.
    public func waitForFrame() -> UnsafePointer<MJFrameData>? {
        mjc_runtime_wait_for_frame(handle)
    }

    /// Get the current frame count (for tracking new frames)
    public var frameCount: UInt64 {
        mjc_runtime_get_frame_count(handle)
    }
}
