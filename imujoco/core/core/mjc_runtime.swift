// mjc_runtime.swift
// Swift wrapper for MJSimulationRuntime C++ class

import Foundation
import MJCPhysicsRuntime

// MARK: - Runtime State

public enum MJRuntimeSimulationState: Int32 {
    case inactive = 0
    case loaded = 1
    case running = 2
    case paused = 3

    public init(from cppState: MJRuntimeState) {
        // C++ enum class is a closed type - all cases are known at compile time
        switch cppState {
        case .Inactive: self = .inactive
        case .Loaded: self = .loaded
        case .Running: self = .running
        case .Paused: self = .paused
        @unknown default:
          <#fatalError()#>
        }
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

    public init(from cppStats: MJRuntimeStats) {
        self.simulationTime = cppStats.simulationTime
        self.measuredSlowdown = cppStats.measuredSlowdown
        self.timestep = cppStats.timestep
        self.stepsPerSecond = cppStats.stepsPerSecond
        self.udpPort = cppStats.udpPort
        self.packetsReceived = cppStats.packetsReceived
        self.packetsSent = cppStats.packetsSent
        self.hasClient = cppStats.hasClient
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
/// ## Lifetime Management
///
/// MJRuntime owns the underlying C++ runtime and destroys it in `deinit`.
/// **Important:** Do not retain `latestFrame` references beyond the MJRuntime's
/// lifetime. Frame data becomes invalid when the runtime is deallocated.
///
/// ## Thread Safety
///
/// **Thread-safe (can be called from any thread):**
/// - `frameCount` - lock-free counter
///
/// **Single-thread only (call from main thread or owner thread):**
/// - `start()`, `pause()`, `reset()`, `step()` - control methods
/// - Camera setters (`cameraAzimuth`, `cameraElevation`, etc.)
/// - `loadModel()`, `unload()`
///
/// **Safe reset sequence:** `pause()` → `reset()` → `start()`
public final class MJRuntime {
    private let runtime: MJSimulationRuntime

    // MARK: - Initialization

    public init(instanceIndex: Int32, targetFPS: Double = 60.0, busyWait: Bool = false, udpPort: UInt16 = 0) throws {
        var config = MJRuntimeConfig()
        config.instanceIndex = instanceIndex
        config.targetFPS = targetFPS
        config.busyWait = busyWait
        config.udpPort = udpPort  // 0 = use default (8888 + instanceIndex)

        guard let ptr = MJSimulationRuntime.create(config) else {
            throw MJRuntimeError.creationFailed
        }
        self.runtime = ptr
    }

    deinit {
        MJSimulationRuntime.destroy(runtime)
    }

    // MARK: - Model Loading

    /// Load a model from an XML file path
    public func loadModel(fromFile path: String) throws {
        var errorString = std.string()
        let success = runtime.loadModel(path, &errorString)

        if !success {
            throw MJRuntimeError.loadFailed(String(errorString))
        }
    }

    /// Load a model from an XML string
    public func loadModel(fromXML xml: String) throws {
        var errorString = std.string()
        let success = runtime.loadModelXML(xml, &errorString)

        if !success {
            throw MJRuntimeError.loadFailed(String(errorString))
        }
    }

    /// Unload the current model
    public func unload() {
        runtime.unload()
    }

    // MARK: - Simulation Control

    /// Start the physics simulation (runs on dedicated C++ thread)
    public func start() {
        runtime.start()
    }

    /// Pause the physics simulation
    public func pause() {
        runtime.pause()
    }

    /// Reset the simulation to initial state
    public func reset() {
        runtime.reset()
    }

    /// Step the simulation manually (when paused)
    public func step() {
        runtime.step()
    }

    // MARK: - State Access

    /// Get the current simulation state
    public var state: MJRuntimeSimulationState {
        MJRuntimeSimulationState(from: runtime.getState())
    }

    /// Get simulation statistics
    public var stats: MJRuntimeStatistics {
        MJRuntimeStatistics(from: runtime.getStats())
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

    // MARK: - Camera Control

    public var cameraAzimuth: Double {
        get { runtime.getCameraAzimuth() }
        set { runtime.setCameraAzimuth(newValue) }
    }

    public var cameraElevation: Double {
        get { runtime.getCameraElevation() }
        set { runtime.setCameraElevation(newValue) }
    }

    public var cameraDistance: Double {
        get { runtime.getCameraDistance() }
        set { runtime.setCameraDistance(newValue) }
    }

    public func setCameraLookat(x: Double, y: Double, z: Double) {
        runtime.setCameraLookat(x, y, z)
    }

    public func resetCamera() {
        runtime.resetCamera()
    }

    // MARK: - Real-time Control

    public var realtimeFactor: Double {
        get { runtime.getRealtimeFactor() }
        set { runtime.setRealtimeFactor(newValue) }
    }

    // MARK: - Frame Access

    /// Get the current frame count (for tracking new frames)
    public var frameCount: UInt64 {
        runtime.getFrameCount()
    }

    /// Get the latest available frame without waiting (non-blocking)
    /// May return the same frame multiple times if physics is slower than render
    /// Returns nil if no frame is available yet
    public var latestFrame: MJFrameData? {
        // MJFrameData is a SWIFT_IMMORTAL_REFERENCE class - Swift treats it as a reference type
        return runtime.getLatestFrame()
    }
}
