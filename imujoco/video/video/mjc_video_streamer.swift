// mjc_video_streamer.swift
// Video streaming orchestrator: capture → encode → transport.
// Runs on a dedicated background thread at configurable FPS.

import Foundation
import Metal
import QuartzCore
import os.log
import MJCPhysicsRuntime
import MJCVideoRuntime

private let logger = Logger(subsystem: "com.mujoco.video", category: "VideoStreamer")

// MARK: - Data Source Protocol

/// Protocol for providing frame data to the video streamer.
/// Implemented by SimulationInstance to feed the capture pipeline.
public protocol MJCVideoDataSource: AnyObject {
    var latestFrame: MJFrameData? { get }
    var meshData: MJMeshData? { get }
    var textureData: MJTextureData? { get }
}

// MARK: - Streamer Configuration

/// Configuration for the video streamer.
public struct MJCVideoStreamerConfig {
    /// Capture width in pixels (default: 256, clamped to 1...4096)
    public var width: Int = 256 {
        didSet { width = max(1, min(4096, width)) }
    }
    /// Capture height in pixels (default: 256, clamped to 1...4096)
    public var height: Int = 256 {
        didSet { height = max(1, min(4096, height)) }
    }
    /// Target capture FPS (default: 10)
    public var targetFPS: Double = 10.0
    /// Video transport port (0 = auto from instance index)
    public var port: UInt16 = 9100
    /// Camera index to stream (default: 0 = free camera)
    public var cameraIndex: UInt8 = 0

    public init() {}
}

// MARK: - Video Streamer

/// Orchestrates video capture from MuJoCo simulations.
///
/// Architecture:
/// 1. Background thread polls `dataSource.latestFrame` at `targetFPS`
/// 2. Offscreen Metal renderer renders the scene to a shared-memory texture
/// 3. Encoder transforms pixels (raw passthrough or JPEG)
/// 4. Transport sends the encoded frame via UDP
///
/// Thread safety: `start()`/`stop()` from any thread. Capture runs on its own thread.
public final class MJCVideoStreamer {
    private let config: MJCVideoStreamerConfig
    private weak var dataSource: MJCVideoDataSource?

    // Capture pipeline (created on capture thread)
    private var offscreenRender: MJCOffscreenRender?
    private let encoder: MJCVideoEncoder
    private let transport: MJVideoUDPTransport

    // Capture thread
    private var captureThread: Thread?
    private var running = false
    private let lock = NSLock()

    // Statistics (capture-thread-only, no synchronization needed)
    private var frameNumber: UInt64 = 0

    // MARK: - Init

    public init(config: MJCVideoStreamerConfig = MJCVideoStreamerConfig(),
                dataSource: MJCVideoDataSource,
                encoder: MJCVideoEncoder = MJCRawEncoder()) {
        self.config = config
        self.dataSource = dataSource
        self.encoder = encoder
        self.transport = MJVideoUDPTransport.create()
    }

    deinit {
        stop()
        MJVideoUDPTransport.destroy(transport)
    }

    // MARK: - Start / Stop

    /// Start video streaming.
    public func start() {
        lock.lock()
        defer { lock.unlock() }
        guard !running else { return }
        running = true

        // Start transport
        guard transport.Start(config.port) else {
            logger.error("Failed to start video transport on port \(self.config.port)")
            running = false
            return
        }

        // Start capture thread
        let thread = Thread { [weak self] in
            self?.captureLoop()
        }
        thread.name = "com.mujoco.video.capture"
        thread.qualityOfService = .userInitiated
        captureThread = thread
        thread.start()

        logger.info("Video streamer started: \(self.config.width)x\(self.config.height) @ \(self.config.targetFPS)fps on port \(self.config.port)")
    }

    /// Stop video streaming.
    public func stop() {
        lock.lock()
        let thread = captureThread
        running = false
        lock.unlock()

        // Wait for capture thread to exit its loop
        if let thread = thread {
            while !thread.isFinished {
                Thread.sleep(forTimeInterval: 0.001)
            }
        }

        lock.lock()
        captureThread = nil
        lock.unlock()

        transport.Stop()
        logger.info("Video streamer stopped")
    }

    /// Whether the streamer is currently running.
    public var isRunning: Bool {
        lock.lock()
        defer { lock.unlock() }
        return running
    }

    /// Whether a receiver is connected.
    public var hasReceiver: Bool {
        transport.HasReceiver()
    }

    // MARK: - Capture Loop

    private func captureLoop() {
        // Create Metal resources on the capture thread
        guard let device = MTLCreateSystemDefaultDevice() else {
            logger.error("No Metal device available for offscreen rendering")
            lock.lock()
            running = false
            lock.unlock()
            return
        }

        do {
            offscreenRender = try MJCOffscreenRender(
                device: device, width: config.width, height: config.height)
        } catch {
            logger.error("Failed to create offscreen renderer: \(error.localizedDescription)")
            lock.lock()
            running = false
            lock.unlock()
            return
        }

        let frameInterval = 1.0 / config.targetFPS

        while true {
            lock.lock()
            let shouldRun = running
            lock.unlock()
            guard shouldRun else { break }

            let frameStart = CACurrentMediaTime()
            captureFrame()
            let elapsed = CACurrentMediaTime() - frameStart

            // Sleep for remaining frame time
            let sleepTime = frameInterval - elapsed
            if sleepTime > 0.001 {
                Thread.sleep(forTimeInterval: sleepTime)
            }
        }

        offscreenRender = nil
    }

    private func captureFrame() {
        guard let dataSource = dataSource,
              let render = offscreenRender else { return }

        // Get latest frame from physics runtime
        guard let frame = dataSource.latestFrame else { return }

        // Render offscreen
        guard let pixelData = render.render(
            frame: frame,
            meshData: dataSource.meshData,
            textureData: dataSource.textureData) else { return }

        // Encode
        guard let encodedData = encoder.encode(
            data: pixelData,
            width: config.width,
            height: config.height) else { return }

        // Build frame descriptor
        frameNumber += 1
        var desc = MJVideoFrameDesc()
        desc.width = UInt32(config.width)
        desc.height = UInt32(config.height)
        desc.stride = UInt32(config.width * 4)
        desc.format = encoder.format
        desc.camera_index = config.cameraIndex
        desc.reserved = (0, 0)
        desc.simulation_time = frame.simulationTime()
        desc.frame_number = frameNumber
        desc.data_size = UInt32(encodedData.count)
        desc.checksum = encodedData.withUnsafeBytes { ptr in
            mjc_video_crc32(ptr.baseAddress!.assumingMemoryBound(to: UInt8.self), encodedData.count)
        }

        // Send via transport
        encodedData.withUnsafeBytes { ptr in
            _ = transport.SendFrame(desc, ptr.baseAddress!.assumingMemoryBound(to: UInt8.self), encodedData.count)
        }
    }
}
