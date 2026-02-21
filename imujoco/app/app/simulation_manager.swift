// simulation_manager.swift
// Main simulation state management using @Observable

import Foundation
import Observation
import os.log
import core
import render
import video
import MJCPhysicsRuntime
#if canImport(UIKit)
import UIKit
#endif
#if canImport(AVFoundation)
import AVFoundation
#endif

private let logger = Logger(subsystem: "com.mujoco.app", category: "SimulationManager")

/// Returns the current process resident memory in megabytes.
func processMemoryMB() -> Double {
    var info = mach_task_basic_info()
    var count = mach_msg_type_number_t(
        MemoryLayout<mach_task_basic_info>.size / MemoryLayout<natural_t>.size
    )
    let result = withUnsafeMutablePointer(to: &info) {
        $0.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
            task_info(mach_task_self_, task_flavor_t(MACH_TASK_BASIC_INFO), $0, &count)
        }
    }
    return result == KERN_SUCCESS ? Double(info.resident_size) / (1024 * 1024) : 0
}

/// Returns the current process CPU usage as a percentage.
/// Can exceed 100% on multi-core (each core contributes up to 100%).
func processCPUUsage() -> Double {
    var threadsList: thread_act_array_t?
    var threadCount = mach_msg_type_number_t(0)

    let result = task_threads(mach_task_self_, &threadsList, &threadCount)
    guard result == KERN_SUCCESS, let threads = threadsList else { return 0 }
    defer {
        vm_deallocate(
            mach_task_self_,
            vm_address_t(bitPattern: threads),
            vm_size_t(Int(threadCount) * MemoryLayout<thread_act_t>.size)
        )
    }

    var totalUsage: Double = 0
    for i in 0..<Int(threadCount) {
        var info = thread_basic_info()
        var count = mach_msg_type_number_t(
            MemoryLayout<thread_basic_info_data_t>.size / MemoryLayout<natural_t>.size
        )
        let kr = withUnsafeMutablePointer(to: &info) {
            $0.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
                thread_info(threads[i], thread_flavor_t(THREAD_BASIC_INFO), $0, &count)
            }
        }
        if kr == KERN_SUCCESS && (info.flags & TH_FLAGS_IDLE) == 0 {
            totalUsage += Double(info.cpu_usage) / Double(TH_USAGE_SCALE) * 100
        }
    }
    return totalUsage
}

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
final class SimulationInstance: Identifiable, MJCRenderDataSource, MJCVideoDataSource, @unchecked Sendable {
    let id: Int
    let port: UInt16

    // C++ physics runtime (owns model, data, scene, camera, option)
    private(set) var runtime: MJRuntime?
    fileprivate(set) var modelName: String = ""

    // UI state (persists across grid/fullscreen switches)
    var isLocked: Bool = true
    var isBlinded: Bool = false
    fileprivate(set) var isLoading: Bool = false
    fileprivate(set) var loadingModelName: String = ""

    // Stored properties for SwiftUI observation
    // Updated periodically from runtime stats (~10Hz polling)
    private(set) var displayTime: Double = 0.0
    private(set) var displaySPS: Int32 = 0
    private(set) var displaySPSFloat: Float = 0.0
    private(set) var displayTXRate: Float = 0.0
    private(set) var displayRXRate: Float = 0.0
    private(set) var displaySceneBrightness: Float = 0.0
    private(set) var displayRenderFPS: Double = 0
    private(set) var displayFrameTime: Double = 0
    private(set) var displayGeomCount: Int32 = 0

    // Written by the render thread at ~60fps; not observation-tracked to avoid
    // triggering SwiftUI updates from the render thread.
    @ObservationIgnored private var _renderedBrightness: Float = 0.0
    @ObservationIgnored private var _renderedFPS: Double = 0
    @ObservationIgnored private var _renderedFrameTime: Double = 0
    @ObservationIgnored private var _renderedGeomCount: Int32 = 0

    // Video streaming (raw UDP for Python driver + MJPEG/HTTP for VLC)
    @ObservationIgnored private var videoStreamer: MJCVideoStreamer?
    @ObservationIgnored private var mjpegStreamer: MJCVideoStreamer?

    // State polling timer
    private var stateUpdateTask: Task<Void, Never>?

    init(id: Int, basePort: UInt16 = 9000) {
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
        mjpegStreamer = nil
        videoStreamer = nil
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

        // Start video streaming
        // Camera port scheme: 9000 + instance_id * 100 + camera_id
        // Cam0 = default free camera
        let cameraPort = UInt16(9000 + id * 100 + 0)

        // Raw UDP streamer for Python driver
        if videoStreamer == nil {
            var config = MJCVideoStreamerConfig()
            config.port = cameraPort
            config.transportMode = .rawUDP
            videoStreamer = MJCVideoStreamer(config: config, dataSource: self)
        }
        videoStreamer?.start()

        // MJPEG/HTTP streamer for VLC (http://<device-ip>:<cameraPort>/)
        // Uses same camera port (TCP vs UDP — no conflict)
        if mjpegStreamer == nil {
            var config = MJCVideoStreamerConfig()
            config.port = cameraPort
            config.transportMode = .mjpegHTTP
            mjpegStreamer = MJCVideoStreamer(config: config, dataSource: self)
        }
        mjpegStreamer?.start()

        // Start state polling for SwiftUI updates
        startStatePolling()
    }

    @MainActor
    func pause() {
        guard let runtime = runtime else { return }
        mjpegStreamer?.stop()
        videoStreamer?.stop()
        runtime.pause()
        stopStatePolling()

        // Update display time one last time
        displayTime = runtime.simulationTime
    }

    /// Stops simulation polling and pauses runtime. Not @MainActor since it's
    /// called from deinit and async contexts. Only performs thread-safe operations.
    func stop() {
        mjpegStreamer?.stop()
        videoStreamer?.stop()
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
        let wasRunning = runtime.state == .running
        if wasRunning {
            pause()  // joins physics thread — safe to mutate data
        }
        runtime.reset()
        displayTime = 0.0
        displaySPS = 0
        displaySPSFloat = 0.0
        displayTXRate = 0.0
        displayRXRate = 0.0
        displaySceneBrightness = 0.0
        displayRenderFPS = 0
        displayFrameTime = 0
        displayGeomCount = 0
        if wasRunning {
            start()
        }
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
                let stats = runtime.stats
                let currentTime = stats.simulationTime
                let currentSPS = stats.stepsPerSecond
                let currentSPSF = stats.stepsPerSecondF
                let currentTXRate = stats.txRate
                let currentRXRate = stats.rxRate
                // Read GPU-computed values (written by render thread, non-observed)
                let currentBrightness = self._renderedBrightness
                let currentRenderFPS = self._renderedFPS
                let currentFrameTime = self._renderedFrameTime
                let currentGeomCount = self._renderedGeomCount
                await MainActor.run {
                    self.displayTime = currentTime
                    self.displaySPS = currentSPS
                    self.displaySPSFloat = currentSPSF
                    self.displayTXRate = currentTXRate
                    self.displayRXRate = currentRXRate
                    self.displaySceneBrightness = currentBrightness
                    self.displayRenderFPS = currentRenderFPS
                    self.displayFrameTime = currentFrameTime
                    self.displayGeomCount = currentGeomCount
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
        displaySPS
    }

    var stepsPerSecondFloat: Float {
        displaySPSFloat
    }

    var txRate: Float {
        displayTXRate
    }

    var rxRate: Float {
        displayRXRate
    }

    var sceneBrightness: Float {
        displaySceneBrightness
    }

    var renderFPS: Double { displayRenderFPS }
    var frameTime: Double { displayFrameTime }
    var geomCount: Int32 { displayGeomCount }

    /// Camera port for this instance (9000 + id * 100 + camera_id).
    /// Cam0 = default free camera.
    var cameraPort: UInt16 {
        UInt16(9000 + id * 100 + 0)
    }

    /// Whether video streaming is active (streamers are running).
    var isStreaming: Bool {
        videoStreamer?.isRunning == true || mjpegStreamer?.isRunning == true
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

    public var meshData: MJMeshData? {
        runtime?.meshData
    }

    public var textureData: MJTextureData? {
        runtime?.textureData
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

    public var cameraLookatX: Double {
        get { runtime?.cameraLookatX ?? 0.0 }
        set { runtime?.cameraLookatX = newValue }
    }

    public var cameraLookatY: Double {
        get { runtime?.cameraLookatY ?? 0.0 }
        set { runtime?.cameraLookatY = newValue }
    }

    public var cameraLookatZ: Double {
        get { runtime?.cameraLookatZ ?? 0.5 }
        set { runtime?.cameraLookatZ = newValue }
    }

    public var renderedSceneBrightness: Float {
        get { _renderedBrightness }
        set { _renderedBrightness = newValue }
    }

    public var renderedFPS: Double {
        get { _renderedFPS }
        set { _renderedFPS = newValue }
    }

    public var renderedFrameTime: Double {
        get { _renderedFrameTime }
        set { _renderedFrameTime = newValue }
    }

    public var renderedGeomCount: Int32 {
        get { _renderedGeomCount }
        set { _renderedGeomCount = newValue }
    }

    public func resetCamera() {
        runtime?.resetCamera()
    }
}

// MARK: - Bundled Model Descriptor

enum ModelSource: String, CaseIterable {
    case imujoco = "iMuJoCo"
    case mujoco = "MuJoCo"
    case menagerie = "Menagerie"
}

struct BundledModel {
    let name: String         // Display name
    let source: ModelSource  // Model origin/group
    let resource: String     // XML filename without extension
    let subdirectory: String?  // Subdirectory in bundle (nil = flat)
    let keyframe: String?    // Initial keyframe name (nil = default state)
    let timestep: Double?    // Override model timestep (nil = use model default)
    let cameraElevation: Double?  // Override initial camera elevation (nil = use default)
    let cameraAzimuth: Double?    // Override initial camera azimuth (nil = use default)
    let cameraDistance: Double?   // Override initial camera distance (nil = use default)
}

// MARK: - Grid Manager

/// Manages 2x2 grid of simulation instances.
/// Note: Uses @unchecked Sendable to allow passing to async contexts. All mutable state
/// is accessed from MainActor (SwiftUI views). The instances array contains SimulationInstance
/// objects which handle their own thread-safety for Metal rendering.
@Observable
final class SimulationGridManager: @unchecked Sendable {
    static let gridSize = 4  // 2x2 grid
    static let basePort: UInt16 = 9000

    private(set) var instances: [SimulationInstance]
    private(set) var fullscreenInstanceId: Int? = nil

    var bundledModels: [BundledModel] {
        [
            BundledModel(name: "Car", source: .mujoco, resource: "car", subdirectory: "model/car",
                         keyframe: nil, timestep: nil, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
            BundledModel(name: "Humanoid (Supine)", source: .mujoco, resource: "humanoid_supine", subdirectory: nil,
                         keyframe: "supine", timestep: nil, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
            BundledModel(name: "Simple Pendulum", source: .imujoco, resource: "simple_pendulum", subdirectory: nil,
                         keyframe: "start", timestep: nil, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
            BundledModel(name: "Agility Cassie", source: .menagerie, resource: "scene", subdirectory: "agility_cassie",
                         keyframe: "home", timestep: nil, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
            BundledModel(name: "ANYmal C", source: .menagerie, resource: "scene", subdirectory: "anybotics_anymal_c",
                         keyframe: nil, timestep: nil, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
            BundledModel(name: "Unitree G1", source: .menagerie, resource: "scene", subdirectory: "unitree_g1",
                         keyframe: nil, timestep: nil, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
            BundledModel(name: "Unitree H1", source: .menagerie, resource: "scene", subdirectory: "unitree_h1",
                         keyframe: nil, timestep: nil, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
        ]
    }

    var bundledModelNames: [String] {
        bundledModels.map { $0.name }
    }

    var bundledModelsBySource: [(source: ModelSource, models: [BundledModel])] {
        ModelSource.allCases.compactMap { source in
            let models = bundledModels.filter { $0.source == source }
            return models.isEmpty ? nil : (source, models)
        }
    }

    init() {
        instances = (1...Self.gridSize).map { index in
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

    @MainActor
    func loadModel(at index: Int, fromFile path: String) async throws {
        guard let instance = instance(at: index) else { return }
        instance.loadingModelName = URL(fileURLWithPath: path).deletingPathExtension().lastPathComponent
        instance.isLoading = true
        do {
            try await instance.loadModel(fromFile: path)
            instance.isLocked = UserDefaults.standard.object(forKey: "defaultLocked") as? Bool ?? true
            instance.start()
            instance.isLoading = false
        } catch {
            instance.isLoading = false
            throw error
        }
    }

    @MainActor
    func loadBundledModel(at index: Int, name: String) async throws {
        guard let instance = instance(at: index) else { return }
        guard let model = bundledModels.first(where: { $0.name == name }) else {
            throw MuJoCoError.loadFailed("Unknown model '\(name)'")
        }

        instance.loadingModelName = name
        instance.isLoading = true

        do {
            // Try multiple search paths for the model file.
            // Rationale: Bundle resource locations can vary between Xcode configurations
            // (Debug/Release), build systems, and test environments. This multi-path
            // approach ensures robustness during development.
            let searchPaths: [(String?, String)] = [
                (model.subdirectory, "model subdirectory"),
                (nil, "Bundle root"),
                ("Resources/Models", "Resources/Models subdirectory"),
                ("Models", "Models subdirectory"),
            ]

            var path: String?
            for (subdirectory, description) in searchPaths {
                if let foundPath = Bundle.main.path(forResource: model.resource, ofType: "xml", inDirectory: subdirectory) {
                    path = foundPath
                    logger.debug("Found model '\(name)' in \(description): \(foundPath)")
                    break
                }
            }

            guard let modelPath = path else {
                let triedPaths = searchPaths.map { $0.1 }.joined(separator: ", ")
                logger.error("Model '\(name)' not found. Tried: \(triedPaths)")
                if let bundlePath = Bundle.main.resourcePath {
                    logger.debug("Bundle resource path: \(bundlePath)")
                    if let contents = try? FileManager.default.contentsOfDirectory(atPath: bundlePath) {
                        logger.debug("Bundle contents: \(contents.prefix(20))")
                    }
                }
                throw MuJoCoError.loadFailed("Model '\(name)' not found in bundle")
            }

            try await instance.loadModel(fromFile: modelPath)
            if let keyframe = model.keyframe {
                instance.runtime?.resetToKeyframe(keyframe)
            }
            if let ts = model.timestep {
                instance.runtime?.setTimestep(ts)
            }
            if let el = model.cameraElevation { instance.cameraElevation = el }
            if let az = model.cameraAzimuth { instance.cameraAzimuth = az }
            if let dist = model.cameraDistance { instance.cameraDistance = dist }
            instance.modelName = name
            instance.isLocked = UserDefaults.standard.object(forKey: "defaultLocked") as? Bool ?? true
            instance.start()
            instance.isLoading = false
        } catch {
            instance.isLoading = false
            throw error
        }
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

    // MARK: - Pause / Resume All

    /// Tracks which instances were running before pauseAll, so resumeAll restores only those.
    private var pausedIndices: [Int] = []

    @MainActor
    func pauseAll() {
        guard pausedIndices.isEmpty else { return }
        pausedIndices = instances.indices.filter { instances[$0].state == .running }
        for i in pausedIndices {
            instances[i].pause()
        }
    }

    @MainActor
    func resumeAll() {
        for i in pausedIndices where i < instances.count {
            instances[i].start()
        }
        pausedIndices = []
    }

    // MARK: - Background Execution

    #if os(iOS)
    private var backgroundTaskID: UIBackgroundTaskIdentifier = .invalid

    @MainActor
    func beginBackgroundExecution() {
        guard backgroundTaskID == .invalid else { return }

        let hasRunning = instances.contains { $0.state == .running }
        guard hasRunning else {
            logger.debug("No running simulations, skipping background task")
            return
        }

        backgroundTaskID = UIApplication.shared.beginBackgroundTask(
            withName: "iMuJoCo Physics"
        ) { [weak self] in
            Task { @MainActor in
                logger.info("Background task expiring")
                self?.endBackgroundExecution()
            }
        }

        if backgroundTaskID != .invalid {
            logger.info("Background task started")
        }
    }

    @MainActor
    func endBackgroundExecution() {
        guard backgroundTaskID != .invalid else { return }
        UIApplication.shared.endBackgroundTask(backgroundTaskID)
        backgroundTaskID = .invalid
    }
    // MARK: - Caffeine Background (silent audio keep-alive)

    private var silentPlayer: AVAudioPlayer?

    @MainActor
    func beginCaffeineBackground() {
        guard silentPlayer == nil else { return }

        let hasRunning = instances.contains { $0.state == .running }
        guard hasRunning else {
            logger.debug("No running simulations, skipping caffeine background")
            return
        }

        do {
            let session = AVAudioSession.sharedInstance()
            try session.setCategory(.playback, options: .mixWithOthers)
            try session.setActive(true)

            let player = try AVAudioPlayer(data: Self.silentWAV)
            player.numberOfLoops = -1  // loop forever
            player.volume = 0
            player.prepareToPlay()

            if player.play() {
                silentPlayer = player
                logger.info("Caffeine background audio started")
            } else {
                logger.error("Caffeine background audio failed: AVAudioPlayer.play() returned false")
                player.stop()
                try? session.setActive(false, options: .notifyOthersOnDeactivation)
            }
        } catch {
            logger.error("Caffeine background audio failed: \(error.localizedDescription)")
            try? AVAudioSession.sharedInstance().setActive(false, options: .notifyOthersOnDeactivation)
        }
    }

    @MainActor
    func endCaffeineBackground() {
        guard let player = silentPlayer else { return }
        player.stop()
        silentPlayer = nil
        try? AVAudioSession.sharedInstance().setActive(false, options: .notifyOthersOnDeactivation)
        logger.info("Caffeine background audio stopped")
    }

    /// Minimal 1-second silent WAV (44100 Hz, mono, 16-bit).
    private static let silentWAV: Data = {
        let sampleRate: UInt32 = 44100
        let numSamples: UInt32 = sampleRate
        let dataSize = numSamples * 2  // 16-bit mono
        var d = Data(capacity: 44 + Int(dataSize))
        func append16(_ v: UInt16) { withUnsafeBytes(of: v.littleEndian) { d.append(contentsOf: $0) } }
        func append32(_ v: UInt32) { withUnsafeBytes(of: v.littleEndian) { d.append(contentsOf: $0) } }
        d.append(contentsOf: [0x52,0x49,0x46,0x46])  // "RIFF"
        append32(36 + dataSize)
        d.append(contentsOf: [0x57,0x41,0x56,0x45])  // "WAVE"
        d.append(contentsOf: [0x66,0x6D,0x74,0x20])  // "fmt "
        append32(16); append16(1); append16(1)        // PCM, mono
        append32(sampleRate); append32(sampleRate * 2) // byte rate
        append16(2); append16(16)                      // block align, bits
        d.append(contentsOf: [0x64,0x61,0x74,0x61])  // "data"
        append32(dataSize)
        d.append(Data(count: Int(dataSize)))           // silence
        return d
    }()

    #else
    func beginBackgroundExecution() {}
    func endBackgroundExecution() {}
    func beginCaffeineBackground() {}
    func endCaffeineBackground() {}
    #endif
}
