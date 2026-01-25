// simulation_manager.swift
// Main simulation state management using @Observable

import Foundation
import Observation
import os.log
import core
import render
import MJCPhysicsRuntime

private let logger = Logger(subsystem: "com.mujoco.app", category: "SimulationManager")

// MARK: - Simulation State

enum SimulationState: Equatable {
    case inactive
    case loaded
    case running
    case paused

    init(from runtimeState: MJRuntimeSimulationState) {
        switch runtimeState {
        case .inactive: self = .inactive
        case .loaded: self = .loaded
        case .running: self = .running
        case .paused: self = .paused
        @unknown default: self = .inactive
        }
    }
}

// MARK: - MuJoCo Error

enum MuJoCoError: Error, LocalizedError {
    case loadFailed(String)

    var errorDescription: String? {
        switch self {
        case .loadFailed(let message): return "Failed to load model: \(message)"
        }
    }
}

// MARK: - Simulation Instance

/// Simulation instance wrapping MJRuntime for physics and rendering.
/// Note: Uses @unchecked Sendable because MJCRenderDataSource protocol is accessed from both
/// main thread (SwiftUI/gestures) and render thread (Metal draw). The underlying C++ runtime
/// is thread-safe for these operations. Observable state (displayTime, modelName) is modified
/// via @MainActor-isolated methods to ensure thread-safe UI updates.
@Observable
final class SimulationInstance: Identifiable, MJCRenderDataSource, @unchecked Sendable {
    let id: Int
    let port: UInt16

    // C++ physics runtime (owns model, data, scene, camera, option)
    private(set) var runtime: MJRuntime?
    private(set) var modelName: String = ""

    // Display time - stored property for SwiftUI observation
    // Updated periodically from runtime stats
    private(set) var displayTime: Double = 0.0

    // State polling timer
    private var stateUpdateTask: Task<Void, Never>?

    init(id: Int, basePort: UInt16 = 8888) {
        self.id = id
        self.port = basePort + UInt16(id)
    }

    deinit {
        stop()
    }

    // MARK: - Model Loading

    func loadModel(fromFile path: String) async throws {
        stop()

        // Create new runtime instance with port for UDP server
        let rt = try MJRuntime(instanceIndex: Int32(id), udpPort: port)
        try rt.loadModel(fromFile: path)

        await MainActor.run {
            self.runtime = rt
            self.modelName = URL(fileURLWithPath: path).deletingPathExtension().lastPathComponent
            self.displayTime = 0.0
        }
    }

    func loadModel(fromXML xml: String, name: String) async throws {
        stop()

        // Create new runtime instance with port for UDP server
        let rt = try MJRuntime(instanceIndex: Int32(id), udpPort: port)
        try rt.loadModel(fromXML: xml)

        await MainActor.run {
            self.runtime = rt
            self.modelName = name
            self.displayTime = 0.0
        }
    }

    @MainActor
    func unload() {
        stop()
        runtime?.unload()
        runtime = nil
        modelName = ""
        displayTime = 0.0
    }

    // MARK: - Simulation Control

    @MainActor
    func start() {
        guard let runtime = runtime else { return }
        guard runtime.state != .running else { return }

        // Start physics on C++ thread
        runtime.start()

        // Start state polling for SwiftUI updates
        startStatePolling()
    }

    @MainActor
    func pause() {
        guard let runtime = runtime else { return }
        runtime.pause()
        stopStatePolling()

        // Update display time one last time
        displayTime = runtime.simulationTime
    }

    /// Stops simulation polling and pauses runtime. Not @MainActor since it's
    /// called from deinit and async contexts. Only performs thread-safe operations.
    func stop() {
        stopStatePolling()
        runtime?.pause()
    }

    @MainActor
    func step() {
        guard let runtime = runtime, runtime.state != .running else { return }
        runtime.step()
        displayTime = runtime.simulationTime
    }

    @MainActor
    func reset() {
        guard let runtime = runtime else { return }
        runtime.reset()
        displayTime = 0.0
    }

    @MainActor
    func togglePlayPause() {
        guard let runtime = runtime else { return }

        if runtime.state == .running {
            pause()
        } else {
            start()
        }
    }

    // MARK: - State Polling

    private func startStatePolling() {
        stateUpdateTask = Task { [weak self] in
            while !Task.isCancelled {
                guard let self = self, let runtime = self.runtime else { break }

                // Update display time periodically (~10Hz to reduce overhead)
                let currentTime = runtime.simulationTime
                await MainActor.run {
                    self.displayTime = currentTime
                }

                try? await Task.sleep(nanoseconds: 100_000_000)  // 100ms
            }
        }
    }

    private func stopStatePolling() {
        stateUpdateTask?.cancel()
        stateUpdateTask = nil
    }

    // MARK: - Properties

    var state: SimulationState {
        guard let runtime = runtime else { return .inactive }
        return SimulationState(from: runtime.state)
    }

    var simulationTime: Double {
        displayTime
    }

    var isActive: Bool {
        runtime != nil
    }

    var stateDescription: String {
        switch state {
        case .inactive: return "Inactive"
        case .loaded: return "Loaded"
        case .running: return "Running"
        case .paused: return "Paused"
        }
    }

    var timestep: Double {
        runtime?.timestep ?? 0.002
    }

    var stepsPerSecond: Int32 {
        runtime?.stats.stepsPerSecond ?? 0
    }

    // MARK: - Network Status

    var hasClient: Bool {
        runtime?.hasClient ?? false
    }

    var packetsReceived: UInt32 {
        runtime?.packetsReceived ?? 0
    }

    var packetsSent: UInt32 {
        runtime?.packetsSent ?? 0
    }

    // MARK: - MJCRenderDataSource Protocol

    public var latestFrame: MJFrameData? {
        runtime?.latestFrame
    }

    public var cameraAzimuth: Double {
        get { runtime?.cameraAzimuth ?? 90.0 }
        set { runtime?.cameraAzimuth = newValue }
    }

    public var cameraElevation: Double {
        get { runtime?.cameraElevation ?? -15.0 }
        set { runtime?.cameraElevation = newValue }
    }

    public var cameraDistance: Double {
        get { runtime?.cameraDistance ?? 3.0 }
        set { runtime?.cameraDistance = newValue }
    }

    public func resetCamera() {
        runtime?.resetCamera()
    }
}

// MARK: - Grid Manager

/// Manages 2x2 grid of simulation instances.
/// Note: Uses @unchecked Sendable to allow passing to async contexts. All mutable state
/// is accessed from MainActor (SwiftUI views). The instances array contains SimulationInstance
/// objects which handle their own thread-safety for Metal rendering.
@Observable
final class SimulationGridManager: @unchecked Sendable {
    static let gridSize = 4  // 2x2 grid
    static let basePort: UInt16 = 8888

    private(set) var instances: [SimulationInstance]
    private(set) var fullscreenInstanceId: Int? = nil

    var bundledModelNames: [String] {
        ["humanoid", "pendulum", "simple_pendulum"]
    }

    init() {
        instances = (0..<Self.gridSize).map { index in
            SimulationInstance(id: index, basePort: Self.basePort)
        }
    }

    // MARK: - Instance Access

    func instance(at index: Int) -> SimulationInstance? {
        guard index >= 0, index < instances.count else { return nil }
        return instances[index]
    }

    var activeInstances: [SimulationInstance] {
        instances.filter { $0.isActive }
    }

    var activeCount: Int {
        activeInstances.count
    }

    // MARK: - Model Loading

    func loadModel(at index: Int, fromFile path: String) async throws {
        guard let instance = instance(at: index) else { return }
        try await instance.loadModel(fromFile: path)
        instance.start()
    }

    @MainActor
    func loadBundledModel(at index: Int, name: String) async throws {
        guard let instance = instance(at: index) else { return }

        // Try multiple search paths for the model file
        let searchPaths: [(String?, String)] = [
            (nil, "Bundle root"),
            ("Resources/Models", "Resources/Models subdirectory"),
            ("Models", "Models subdirectory")
        ]

        var path: String?
        for (subdirectory, description) in searchPaths {
            if let foundPath = Bundle.main.path(forResource: name, ofType: "xml", inDirectory: subdirectory) {
                path = foundPath
                logger.debug("Found model '\(name)' in \(description): \(foundPath)")
                break
            }
        }

        guard let modelPath = path else {
            // Log bundle contents for debugging (only at debug level)
            if let bundlePath = Bundle.main.resourcePath {
                logger.debug("Bundle resource path: \(bundlePath)")
                if let contents = try? FileManager.default.contentsOfDirectory(atPath: bundlePath) {
                    logger.debug("Bundle contents: \(contents.prefix(20))")
                }
            }
            throw MuJoCoError.loadFailed("Model '\(name)' not found in bundle")
        }

        // Verify file exists before passing to MuJoCo
        if !FileManager.default.fileExists(atPath: modelPath) {
            throw MuJoCoError.loadFailed("Model file not found at path: \(modelPath)")
        }

        try await instance.loadModel(fromFile: modelPath)
        instance.start()
    }

    @MainActor
    func unload(at index: Int) {
        instance(at: index)?.unload()
    }

    // MARK: - Fullscreen

    @MainActor
    func enterFullscreen(index: Int) {
        fullscreenInstanceId = index
    }

    @MainActor
    func exitFullscreen() {
        fullscreenInstanceId = nil
    }

    var isFullscreen: Bool {
        fullscreenInstanceId != nil
    }

    var fullscreenInstance: SimulationInstance? {
        guard let id = fullscreenInstanceId else { return nil }
        return instance(at: id)
    }

    // MARK: - First Available

    var firstAvailableIndex: Int? {
        instances.firstIndex { !$0.isActive }
    }
}
