// simulation_manager.swift
// Main simulation state management using @Observable

import Foundation
import Observation
import os.log
import core
import render
import video
import MJCPhysicsRuntime
import MJCGrpcServer
#if canImport(UIKit)
import UIKit
#endif
#if canImport(BackgroundTasks)
import BackgroundTasks
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
    private(set) var port: UInt16
    private(set) var cameraBasePort: UInt16

    // C++ physics runtime (owns model, data, scene, camera, option)
    private(set) var runtime: MJRuntime?
    fileprivate(set) var modelName: String = ""
    /// Path of the currently loaded model file (for live port rebind reload).
    @ObservationIgnored private(set) var modelPath: String?

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

    // Video streaming (MJPEG/RTSP/QUIC for VLC, browsers, custom receivers)
    @ObservationIgnored private var vlcStreamer: MJCVideoStreamer?

    /// Current VLC transport mode for this instance (observable for UI toggle).
    private(set) var vlcTransportMode: MJCVideoTransportMode = .mjpegHTTP
    /// Whether VLC streaming is disabled ("Off" mode).
    private(set) var vlcOff: Bool = false
    /// Whether the user has explicitly toggled the transport mode on this instance.
    @ObservationIgnored private var vlcTransportModeExplicit = false
    /// Guard against overlapping VLC streamer restarts (observable for UI disable).
    private(set) var isRestartingVLC = false

    // State polling timer
    private var stateUpdateTask: Task<Void, Never>?

    init(id: Int, udpPort: UInt16, cameraPort: UInt16) {
        self.id = id
        self.port = udpPort
        self.cameraBasePort = cameraPort
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
            self.modelPath = path
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
            self.modelPath = nil  // XML-loaded models don't have a file path
            self.modelName = name
            self.displayTime = 0.0
        }
    }

    @MainActor
    func unload() {
        stop()
        vlcStreamer = nil
        runtime?.unload()
        runtime = nil
        modelPath = nil
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
        // Camera port from per-instance config (default: 9000 + id * 100)
        // Cam0 = default free camera
        let cameraPort = cameraBasePort

        // VLC-facing streamer (MJPEG/HTTP, RTP/RTSP, or HEVC/QUIC per user setting)
        // All modes use cameraPort directly; RTP (UDP) + RTSP (TCP) share the port number
        if vlcStreamer == nil {
            if !vlcTransportModeExplicit {
                let setting = UserDefaults.standard.integer(forKey: "videoTransport")
                switch setting {
                case 1: vlcTransportMode = .rtpRTSP; vlcOff = false
                case 2: vlcTransportMode = .hevcQUIC; vlcOff = false
                case 3: vlcOff = true
                default: vlcTransportMode = .mjpegHTTP; vlcOff = false
                }
            }
            if !vlcOff {
                var config = MJCVideoStreamerConfig()
                config.port = cameraPort
                config.transportMode = vlcTransportMode
                config.rtspPort = cameraPort
                vlcStreamer = MJCVideoStreamer(config: config, dataSource: self)
            }
        }
        vlcStreamer?.start()

        // Start state polling for SwiftUI updates
        startStatePolling()
    }

    @MainActor
    func pause() {
        guard let runtime = runtime else { return }
        vlcStreamer?.stop()
        runtime.pause()
        stopStatePolling()

        // Update display time one last time
        displayTime = runtime.simulationTime
    }

    /// Stops simulation polling and pauses runtime. Not @MainActor since it's
    /// called from deinit and async contexts. Only performs thread-safe operations.
    func stop() {
        vlcStreamer?.stop()
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

    /// Camera port for this instance (configured via UserDefaults, default: 9000 + id * 100).
    /// Cam0 = default free camera.
    var cameraPort: UInt16 {
        cameraBasePort
    }

    /// Whether video streaming is active (streamers are running).
    var isStreaming: Bool {
        vlcStreamer?.isRunning == true
    }

    /// Measured output FPS of the active video streamer (updated once per second).
    var videoFPS: Double {
        vlcStreamer?.measuredFPS ?? 0
    }

    /// Suspend GPU-based video capture (app entering background).
    /// Transports stay alive so clients remain connected.
    func suspendVideoCapture() {
        vlcStreamer?.suspend()
    }

    /// Resume GPU-based video capture (app returning to foreground).
    func resumeVideoCapture() {
        vlcStreamer?.resume()
    }

    /// Restart the VLC-facing streamer with a new transport mode.
    /// Only acts if the instance is currently running.
    /// Moves the blocking stop() off MainActor to avoid freezing the UI.
    @MainActor
    func restartVLCStreamer(mode: MJCVideoTransportMode) {
        let modeStr: String
        switch mode {
        case .mjpegHTTP: modeStr = "MJPEG/HTTP"
        case .rtpRTSP: modeStr = "RTP/RTSP"
        case .hevcQUIC: modeStr = "HEVC/QUIC"
        case .rawUDP: modeStr = "rawUDP"
        }
        guard state == .running, !isRestartingVLC else {
            logger.debug("restartVLCStreamer(\(modeStr)): skipped, instance \(self.id) state=\(self.stateDescription) restarting=\(self.isRestartingVLC)")
            return
        }
        isRestartingVLC = true
        logger.info("restartVLCStreamer(\(modeStr)): restarting VLC streamer on instance \(self.id)")

        // Capture the old streamer and detach from instance immediately.
        // The local var keeps it alive until the background task finishes.
        let oldStreamer = vlcStreamer
        vlcStreamer = nil
        vlcTransportMode = mode
        vlcOff = false
        vlcTransportModeExplicit = true

        Task.detached { [weak self] in
            // stop() spin-waits for the capture thread — do it off MainActor.
            oldStreamer?.stop()

            await MainActor.run { [weak self] in
                guard let self else { return }
                defer { self.isRestartingVLC = false }
                guard self.state == .running else { return }

                var config = MJCVideoStreamerConfig()
                config.port = self.cameraPort
                config.transportMode = mode
                config.rtspPort = self.cameraPort
                self.vlcStreamer = MJCVideoStreamer(config: config, dataSource: self)
                self.vlcStreamer?.start()
            }
        }
    }

    /// Stop the VLC-facing streamer (set to "Off" mode).
    @MainActor
    func stopVLCStreamer() {
        guard !isRestartingVLC else { return }
        isRestartingVLC = true
        let oldStreamer = vlcStreamer
        vlcStreamer = nil
        vlcOff = true
        vlcTransportModeExplicit = true

        Task.detached { [weak self] in
            oldStreamer?.stop()
            await MainActor.run { [weak self] in
                self?.isRestartingVLC = false
            }
        }
    }

    /// Cycle the VLC transport mode: Off → MJPEG/HTTP → RTP/RTSP → HEVC/QUIC → Off.
    @MainActor
    func toggleVLCTransport() {
        if vlcOff {
            restartVLCStreamer(mode: .mjpegHTTP)
        } else {
            switch vlcTransportMode {
            case .mjpegHTTP: restartVLCStreamer(mode: .rtpRTSP)
            case .rtpRTSP: restartVLCStreamer(mode: .hevcQUIC)
            case .hevcQUIC: stopVLCStreamer()
            case .rawUDP: restartVLCStreamer(mode: .mjpegHTTP)
            }
        }
    }

    // MARK: - Live Port Rebinding

    /// Rebind the camera (video) port without stopping physics.
    /// Moves blocking stop() off MainActor to avoid freezing the UI.
    @MainActor
    func rebindCameraPort(_ newPort: UInt16) {
        guard newPort != cameraBasePort else { return }
        let wasRunning = runtime?.state == .running
        logger.info("Instance \(self.id): rebinding camera port \(self.cameraBasePort) → \(newPort)")

        // Capture old streamer; detach from instance immediately.
        let oldVLC = vlcStreamer
        vlcStreamer = nil

        // Update port
        cameraBasePort = newPort

        Task.detached { [weak self] in
            // Stop on background to avoid blocking MainActor's spin-wait.
            oldVLC?.stop()

            await MainActor.run { [weak self] in
                guard let self, wasRunning else { return }

                var vlcConfig = MJCVideoStreamerConfig()
                vlcConfig.port = self.cameraBasePort
                vlcConfig.transportMode = self.vlcTransportMode
                vlcConfig.rtspPort = self.cameraBasePort
                self.vlcStreamer = MJCVideoStreamer(config: vlcConfig, dataSource: self)
                self.vlcStreamer?.start()
            }
        }
    }

    /// Rebind the UDP (physics control) port. Requires runtime destruction and reload.
    @MainActor
    func rebindUDPPort(_ newPort: UInt16) async {
        guard newPort != port else { return }
        logger.info("Instance \(self.id): rebinding UDP port \(self.port) → \(newPort)")

        let savedPath = modelPath
        let savedModelName = modelName
        let wasRunning = runtime?.state == .running
        let savedLocked = isLocked

        // Tear down
        stop()
        vlcStreamer = nil
        runtime?.unload()
        runtime = nil

        // Update port
        port = newPort

        // Reload model if one was loaded
        if let path = savedPath {
            do {
                let rt = try MJRuntime(instanceIndex: Int32(id), udpPort: port)
                try rt.loadModel(fromFile: path)
                runtime = rt
                modelPath = path
                modelName = savedModelName
                displayTime = 0.0
                isLocked = savedLocked
                if wasRunning { start() }
            } catch {
                logger.error("Failed to reload model after UDP rebind: \(error.localizedDescription)")
                modelPath = nil
                modelName = ""
            }
        }
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
    case unitreeRLGym = "Unitree RL Gym"
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

// MARK: - gRPC Callbacks (file-level @convention(c)-compatible functions)

/// Semaphore timeout for gRPC → MainActor callbacks (seconds).
/// LoadModel/Unload may take longer (model parsing, mesh extraction).
private let grpcCallbackTimeout: DispatchTimeInterval = .seconds(10)
private let grpcLoadCallbackTimeout: DispatchTimeInterval = .seconds(120)

private func grpcLoadModelHandler(_ instanceId: Int32, _ modelName: UnsafePointer<CChar>?, _ context: UnsafeMutableRawPointer?) -> Bool {
    guard let context = context, let modelName = modelName else { return false }
    let manager = Unmanaged<SimulationGridManager>.fromOpaque(context).takeUnretainedValue()
    let name = String(cString: modelName)
    let arrayIndex = Int(instanceId) - 1

    var success = false
    let semaphore = DispatchSemaphore(value: 0)
    Task { @MainActor in
        do {
            try await manager.loadBundledModel(at: arrayIndex, name: name)
            success = true
        } catch {
            logger.error("gRPC LoadModel failed: \(error.localizedDescription)")
        }
        semaphore.signal()
    }
    if semaphore.wait(timeout: .now() + grpcLoadCallbackTimeout) == .timedOut {
        logger.error("gRPC LoadModel callback timed out for instance \(instanceId)")
        return false
    }
    return success
}

private func grpcUnloadHandler(_ instanceId: Int32, _ context: UnsafeMutableRawPointer?) -> Bool {
    guard let context = context else { return false }
    let manager = Unmanaged<SimulationGridManager>.fromOpaque(context).takeUnretainedValue()
    let arrayIndex = Int(instanceId) - 1

    var success = false
    let semaphore = DispatchSemaphore(value: 0)
    Task { @MainActor in
        guard let instance = manager.instance(at: arrayIndex), instance.runtime != nil else {
            semaphore.signal()
            return
        }
        manager.unload(at: arrayIndex)
        success = true
        semaphore.signal()
    }
    if semaphore.wait(timeout: .now() + grpcLoadCallbackTimeout) == .timedOut {
        logger.error("gRPC Unload callback timed out for instance \(instanceId)")
        return false
    }
    return success
}

private func grpcOperationHandler(_ instanceId: Int32, _ op: Int32, _ param: Int32, _ context: UnsafeMutableRawPointer?) -> Bool {
    guard let context = context else { return false }
    let manager = Unmanaged<SimulationGridManager>.fromOpaque(context).takeUnretainedValue()
    let arrayIndex = Int(instanceId) - 1

    var success = false
    let semaphore = DispatchSemaphore(value: 0)
    Task { @MainActor in
        guard let instance = manager.instance(at: arrayIndex), instance.runtime != nil else {
            semaphore.signal()
            return
        }
        switch op {
        case MJ_GRPC_OP_START.rawValue:
            instance.start()
            success = true
        case MJ_GRPC_OP_PAUSE.rawValue:
            instance.pause()
            success = true
        case MJ_GRPC_OP_RESET.rawValue:
            instance.reset()
            success = true
        case MJ_GRPC_OP_STEP.rawValue:
            instance.step()
            success = true
        case MJ_GRPC_OP_RESET_TO_KEYFRAME.rawValue:
            let wasRunning = instance.state == .running
            if wasRunning { instance.pause() }
            success = instance.runtime?.resetToKeyframe(index: param) ?? false
            if wasRunning { instance.start() }
        default:
            logger.error("grpcOperationHandler: unknown op \(op)")
        }
        semaphore.signal()
    }
    if semaphore.wait(timeout: .now() + grpcCallbackTimeout) == .timedOut {
        logger.error("gRPC operation callback timed out for instance \(instanceId)")
        return false
    }
    return success
}

private func grpcGetStateHandler(_ instanceId: Int32, _ outState: UnsafeMutablePointer<MJGrpcPhysicsState>?, _ context: UnsafeMutableRawPointer?) -> Bool {
    guard let context = context, let outState = outState else { return false }
    let manager = Unmanaged<SimulationGridManager>.fromOpaque(context).takeUnretainedValue()
    let arrayIndex = Int(instanceId) - 1

    var success = false
    let semaphore = DispatchSemaphore(value: 0)
    Task { @MainActor in
        guard let instance = manager.instance(at: arrayIndex),
              let cppRuntime = instance.runtime?.cppRuntime else {
            semaphore.signal()
            return
        }
        MJGrpcFillPhysicsState(outState, cppRuntime)
        success = outState.pointee.valid
        semaphore.signal()
    }
    if semaphore.wait(timeout: .now() + grpcCallbackTimeout) == .timedOut {
        logger.error("gRPC GetState callback timed out for instance \(instanceId)")
        return false
    }
    return success
}

private func grpcGetInstanceInfoHandler(_ instanceId: Int32, _ outInfo: UnsafeMutablePointer<MJGrpcInstanceInfo>?, _ context: UnsafeMutableRawPointer?) -> Bool {
    guard let context = context, let outInfo = outInfo else { return false }
    let manager = Unmanaged<SimulationGridManager>.fromOpaque(context).takeUnretainedValue()
    let arrayIndex = Int(instanceId) - 1

    var success = false
    let semaphore = DispatchSemaphore(value: 0)
    Task { @MainActor in
        guard let instance = manager.instance(at: arrayIndex),
              let runtime = instance.runtime else {
            semaphore.signal()
            return
        }
        outInfo.pointee.valid = true
        outInfo.pointee.state = runtime.state.rawValue
        let stats = runtime.stats
        outInfo.pointee.stepsPerSecond = stats.stepsPerSecond
        outInfo.pointee.txRate = stats.txRate
        outInfo.pointee.rxRate = stats.rxRate
        outInfo.pointee.packetsSent = stats.packetsSent
        outInfo.pointee.packetsReceived = stats.packetsReceived
        outInfo.pointee.udpPort = stats.udpPort
        outInfo.pointee.hasClient = stats.hasClient
        success = true
        semaphore.signal()
    }
    if semaphore.wait(timeout: .now() + grpcCallbackTimeout) == .timedOut {
        logger.error("gRPC GetInstanceInfo callback timed out for instance \(instanceId)")
        return false
    }
    return success
}

// MARK: - Grid Manager

/// Manages 2x2 grid of simulation instances.
/// Note: Uses @unchecked Sendable to allow passing to async contexts. All mutable state
/// is accessed from MainActor (SwiftUI views). The instances array contains SimulationInstance
/// objects which handle their own thread-safety for Metal rendering.
@Observable
final class SimulationGridManager: @unchecked Sendable {
    static let gridSize = 4  // 2x2 grid

    private(set) var instances: [SimulationInstance]
    private(set) var fullscreenInstanceId: Int? = nil
    @ObservationIgnored private var grpcServer: MJGrpcServer?
    @ObservationIgnored private(set) var grpcPort: UInt16

    /// Monotonically increasing count of gRPC RPCs processed.
    var grpcRpcCount: UInt64 {
        grpcServer?.rpcCount() ?? 0
    }

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
            BundledModel(name: "Unitree G1 (RL)", source: .unitreeRLGym, resource: "scene", subdirectory: "unitree_g1_rl",
                         keyframe: "rl_stand", timestep: 0.002, cameraElevation: nil, cameraAzimuth: nil, cameraDistance: nil),
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

    /// Read a port from UserDefaults, returning `fallback` if missing or out of valid range (1024–65535).
    private static func readPort(_ key: String, fallback: UInt16) -> UInt16 {
        let val = UserDefaults.standard.integer(forKey: key)
        return (val >= 1024 && val <= 65535) ? UInt16(val) : fallback
    }

    init() {
        // Read per-instance ports from UserDefaults (0 or missing = use default)
        let defaultUdpPorts: [UInt16] = [9001, 9002, 9003, 9004]
        let defaultCamPorts: [UInt16] = [9100, 9200, 9300, 9400]

        var insts: [SimulationInstance] = []
        for i in 0..<Self.gridSize {
            let instNum = i + 1
            let udpPort = Self.readPort("inst\(instNum)_udpPort", fallback: defaultUdpPorts[i])
            let camPort = Self.readPort("inst\(instNum)_camPort", fallback: defaultCamPorts[i])
            insts.append(SimulationInstance(id: instNum, udpPort: udpPort, cameraPort: camPort))
        }
        instances = insts
        grpcPort = Self.readPort("grpcPort", fallback: 8999)

        startGrpcServer()
    }

    // MARK: - gRPC Server

    private func startGrpcServer() {
        var config = MJGrpcServerConfig()
        config.port = grpcPort
        config.numInstances = Int32(Self.gridSize)

        guard let server = MJGrpcServer.create(config) else {
            logger.error("Failed to create gRPC server")
            return
        }

        let ctx = Unmanaged.passUnretained(self).toOpaque()
        server.setLoadCallback(grpcLoadModelHandler, ctx)
        server.setUnloadCallback(grpcUnloadHandler, ctx)
        server.setOperationCallback(grpcOperationHandler, ctx)
        server.setGetStateCallback(grpcGetStateHandler, ctx)
        server.setGetInstanceInfoCallback(grpcGetInstanceInfoHandler, ctx)
        server.start()
        grpcServer = server
        logger.info("gRPC server started on port \(self.grpcPort)")
    }

    /// Rebind the gRPC server to a new port. Re-registers all active runtimes.
    func rebindGRPCPort(_ newPort: UInt16) {
        guard newPort != grpcPort else { return }
        logger.info("Rebinding gRPC server: port \(self.grpcPort) → \(newPort)")

        // Unregister all runtimes from old server
        for inst in instances {
            grpcServer?.unregisterRuntime(Int32(inst.id))
        }

        // Destroy old server (SWIFT_IMMORTAL_REFERENCE means Swift won't
        // release C++ objects, so we must call destroy() explicitly).
        if let old = grpcServer {
            MJGrpcServer.destroy(old)
        }
        grpcServer = nil

        // Start new server on new port
        grpcPort = newPort
        startGrpcServer()

        // Re-register active runtimes
        for inst in instances {
            if let rt = inst.runtime {
                grpcServer?.registerRuntime(Int32(inst.id), rt.cppRuntime, inst.modelName)
            }
        }
    }

    /// Rebind an instance's UDP port with gRPC re-registration.
    @MainActor
    func rebindInstanceUDPPort(at index: Int, newPort: UInt16) async {
        guard let instance = instance(at: index) else { return }
        grpcServer?.unregisterRuntime(Int32(instance.id))
        await instance.rebindUDPPort(newPort)
        if let rt = instance.runtime {
            grpcServer?.registerRuntime(Int32(instance.id), rt.cppRuntime, instance.modelName)
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

    /// True if any instance is mid-video-transport switch (UI should disable transport buttons).
    var isRestartingVideoTransport: Bool {
        instances.contains { $0.isRestartingVLC }
    }

    // MARK: - Model Loading

    @MainActor
    func loadModel(at index: Int, fromFile path: String) async throws {
        guard let instance = instance(at: index) else { return }
        grpcServer?.unregisterRuntime(Int32(instance.id))
        instance.loadingModelName = URL(fileURLWithPath: path).deletingPathExtension().lastPathComponent
        instance.isLoading = true
        do {
            try await instance.loadModel(fromFile: path)
            instance.isLocked = UserDefaults.standard.object(forKey: "defaultLocked") as? Bool ?? true
            instance.start()
            if let rt = instance.runtime {
                grpcServer?.registerRuntime(Int32(instance.id), rt.cppRuntime, instance.modelName)
            }
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

        grpcServer?.unregisterRuntime(Int32(instance.id))
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
            if let rt = instance.runtime {
                grpcServer?.registerRuntime(Int32(instance.id), rt.cppRuntime, name)
            }
            instance.isLoading = false
        } catch {
            instance.isLoading = false
            throw error
        }
    }

    @MainActor
    func unload(at index: Int) {
        guard let inst = instance(at: index) else { return }
        grpcServer?.unregisterRuntime(Int32(inst.id))
        inst.unload()
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

    // MARK: - Video Capture Suspend / Resume

    /// Suspend GPU-based video capture on all instances (app backgrounded).
    /// Call this even in caffeine mode — physics can run without GPU, but
    /// Metal command buffers are rejected from background.
    func suspendVideoCapture() {
        for instance in instances {
            instance.suspendVideoCapture()
        }
    }

    /// Resume GPU-based video capture on all instances (app foregrounded).
    func resumeVideoCapture() {
        for instance in instances {
            instance.resumeVideoCapture()
        }
    }

    /// Restart VLC-facing streamers on all running instances with the given transport setting.
    /// `videoTransport`: 0 = MJPEG/HTTP, 1 = RTP/RTSP, 2 = HEVC/QUIC, 3 = Off.
    @MainActor
    func restartVLCStreamers(videoTransport: Int) {
        logger.info("restartVLCStreamers: videoTransport=\(videoTransport)")
        if videoTransport == 3 {
            for instance in instances {
                instance.stopVLCStreamer()
            }
            return
        }
        let mode: MJCVideoTransportMode
        switch videoTransport {
        case 1: mode = .rtpRTSP
        case 2: mode = .hevcQUIC
        default: mode = .mjpegHTTP
        }
        for instance in instances {
            instance.restartVLCStreamer(mode: mode)
        }
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
    // MARK: - Caffeine Background (BGProcessingTask)

    static let caffeineTaskID = "com.hhkblogi.imujoco.app.simulation"

    private var caffeineActive = false

    @MainActor
    func beginCaffeineBackground() {
        guard !caffeineActive else { return }

        let hasRunning = instances.contains { $0.state == .running }
        guard hasRunning else {
            logger.debug("No running simulations, skipping caffeine background")
            return
        }

        caffeineActive = true
        scheduleCaffeineTask()
        logger.info("Caffeine background requested via BGProcessingTask")
    }

    @MainActor
    func endCaffeineBackground() {
        guard caffeineActive else { return }
        caffeineActive = false
        BGTaskScheduler.shared.cancel(taskRequestWithIdentifier: Self.caffeineTaskID)
        logger.info("Caffeine background cancelled")
    }

    private func scheduleCaffeineTask() {
        let request = BGProcessingTaskRequest(identifier: Self.caffeineTaskID)
        request.requiresNetworkConnectivity = false
        request.requiresExternalPower = false
        request.earliestBeginDate = nil  // run as soon as possible
        do {
            try BGTaskScheduler.shared.submit(request)
            logger.debug("BGProcessingTask scheduled")
        } catch {
            logger.error("Failed to schedule BGProcessingTask: \(error.localizedDescription)")
        }
    }

    /// Called when the system launches our BGProcessingTask.
    @MainActor
    func handleCaffeineTask(_ task: BGProcessingTask) {
        logger.info("BGProcessingTask started by system")

        task.expirationHandler = {
            logger.info("BGProcessingTask expiring")
        }

        // Keep the task alive while caffeine is active
        // When done (or app returns to foreground), complete it
        if caffeineActive {
            // Re-schedule for next opportunity
            scheduleCaffeineTask()
        }

        // Mark complete — the simulation threads keep running independently
        // until the process is suspended. Re-scheduling above asks the system
        // to wake us again.
        task.setTaskCompleted(success: true)
    }

    /// Must be called before app finishes launching (e.g. in App.init).
    static func registerBackgroundTask() {
        BGTaskScheduler.shared.register(
            forTaskWithIdentifier: caffeineTaskID,
            using: .main
        ) { task in
            guard let processingTask = task as? BGProcessingTask else { return }
            NotificationCenter.default.post(
                name: .caffeineBackgroundTaskLaunched,
                object: processingTask
            )
        }
    }

    #else
    func beginBackgroundExecution() {}
    func endBackgroundExecution() {}
    func beginCaffeineBackground() {}
    func endCaffeineBackground() {}
    #endif
}

// MARK: - Notification for BGProcessingTask bridging

extension Notification.Name {
    static let caffeineBackgroundTaskLaunched = Notification.Name("caffeineBackgroundTaskLaunched")
}
