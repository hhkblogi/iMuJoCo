// mjc_video_streamer.swift
// Video streaming orchestrator: capture → encode → transport.
// Runs on a dedicated background thread at configurable FPS.

import CoreGraphics
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

/// Transport mode for video streaming.
public enum MJCVideoTransportMode {
    /// Raw UDP with custom fragment protocol (for Python driver)
    case rawUDP
    /// RTP/RTSP with JPEG encoding (for VLC/ffplay)
    case rtpRTSP
    /// MJPEG over HTTP (for VLC/browsers, simpler and more reliable than RTP)
    case mjpegHTTP
}

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
    public var targetFPS: Double = 30.0
    /// Video transport port (camera port scheme: 9000 + instance * 100 + camera)
    public var port: UInt16 = 9100
    /// Camera index to stream (default: 0 = free camera)
    public var cameraIndex: UInt8 = 0
    /// Transport mode
    public var transportMode: MJCVideoTransportMode = .rawUDP
    /// JPEG quality for JPEG-based modes (0.0-1.0, default 0.8)
    public var jpegQuality: CGFloat = 0.8
    /// RTSP server port for rtpRTSP mode (default: 8554)
    public var rtspPort: UInt16 = 8554
    public init() {}
}

// MARK: - Video Streamer

/// Orchestrates video capture from MuJoCo simulations.
///
/// Supports two transport modes:
/// - **rawUDP**: Custom fragment protocol for Python driver (MJCRawEncoder + MJVideoUDPTransport)
/// - **rtpRTSP**: Standard RTP/RTSP for VLC/ffplay (MJCJPEGEncoder + MJVideoRTPTransport + RTSP server)
///
/// Thread safety: `start()`/`stop()` from any thread. Capture runs on its own thread.
public final class MJCVideoStreamer {
    private let config: MJCVideoStreamerConfig
    private weak var dataSource: MJCVideoDataSource?

    // Capture pipeline (created on capture thread)
    private var offscreenRender: MJCOffscreenRender?
    private let encoder: MJCVideoEncoder

    // Transport (one of these is active based on config)
    private let udpTransport: MJVideoUDPTransport?
    private let rtpTransport: MJVideoRTPTransport?
    private let rtspServer: MJVideoRTSPServer?
    private let mjpegServer: MJVideoMJPEGServer?

    // Capture thread
    private var captureThread: Thread?
    private var running = false
    private var suspended = false  // GPU capture paused (app backgrounded)
    private let lock = NSLock()

    // Statistics
    private var frameNumber: UInt64 = 0           // capture-thread-only
    private var fpsFrameCount: UInt64 = 0          // capture-thread-only
    private var fpsLastUpdate: Double = 0          // capture-thread-only
    private var _measuredFPS: Double = 0           // written by capture thread, read by main
    private let fpsLock = NSLock()

    // MARK: - Init

    public init(config: MJCVideoStreamerConfig = MJCVideoStreamerConfig(),
                dataSource: MJCVideoDataSource) {
        self.config = config
        self.dataSource = dataSource

        switch config.transportMode {
        case .rawUDP:
            self.encoder = MJCRawEncoder()
            self.udpTransport = MJVideoUDPTransport.create()
            self.rtpTransport = nil
            self.rtspServer = nil
            self.mjpegServer = nil

        case .rtpRTSP:
            self.encoder = MJCJPEGEncoder(quality: config.jpegQuality)
            self.udpTransport = nil
            let rtp = MJVideoRTPTransport.create()
            self.rtpTransport = rtp
            self.rtspServer = MJVideoRTSPServer.create(rtp)
            self.mjpegServer = nil

        case .mjpegHTTP:
            self.encoder = MJCJPEGEncoder(quality: config.jpegQuality)
            self.udpTransport = nil
            self.rtpTransport = nil
            self.rtspServer = nil
            self.mjpegServer = MJVideoMJPEGServer.create()
        }
    }

    deinit {
        stop()
        if let udp = udpTransport { MJVideoUDPTransport.destroy(udp) }
        // Destroy RTSP server before RTP transport (reverse creation order).
        // MJVideoRTSPServer holds a raw pointer to MJVideoRTPTransport.
        if let rtsp = rtspServer { MJVideoRTSPServer.destroy(rtsp) }
        if let rtp = rtpTransport { MJVideoRTPTransport.destroy(rtp) }
        if let mjpeg = mjpegServer { MJVideoMJPEGServer.destroy(mjpeg) }
    }

    // MARK: - Start / Stop

    /// Start video streaming.
    public func start() {
        lock.lock()
        defer { lock.unlock() }
        guard !running else { return }
        running = true

        // Start transport(s)
        switch config.transportMode {
        case .rawUDP:
            guard let udp = udpTransport, udp.Start(config.port) else {
                logger.error("Failed to start UDP transport on port \(self.config.port)")
                running = false
                return
            }

        case .rtpRTSP:
            let rtpPort = config.port + 2
            guard let rtp = rtpTransport, rtp.Start(rtpPort) else {
                logger.error("Failed to start RTP transport on port \(self.config.port + 2)")
                running = false
                return
            }
            if let rtsp = rtspServer {
                if !rtsp.Start(config.rtspPort,
                               UInt16(config.width),
                               UInt16(config.height)) {
                    logger.error("Failed to start RTSP server on port \(self.config.rtspPort)")
                }
            }

        case .mjpegHTTP:
            guard let mjpeg = mjpegServer, mjpeg.Start(config.port) else {
                logger.error("Failed to start MJPEG server on port \(self.config.port)")
                running = false
                return
            }
        }

        // Start capture thread
        let thread = Thread { [weak self] in
            self?.captureLoop()
        }
        thread.name = "com.mujoco.video.capture"
        thread.qualityOfService = .userInitiated
        captureThread = thread
        thread.start()

        let modeStr: String
        switch config.transportMode {
        case .rawUDP: modeStr = "raw UDP"
        case .rtpRTSP: modeStr = "RTP/RTSP"
        case .mjpegHTTP: modeStr = "MJPEG/HTTP"
        }
        logger.info("Video streamer started: \(self.config.width)x\(self.config.height) @ \(self.config.targetFPS)fps [\(modeStr)] on port \(self.config.port)")
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

        mjpegServer?.Stop()
        rtspServer?.Stop()
        rtpTransport?.Stop()
        udpTransport?.Stop()
        logger.info("Video streamer stopped")
    }

    /// Whether the streamer is currently running.
    public var isRunning: Bool {
        lock.lock()
        defer { lock.unlock() }
        return running
    }

    /// Suspend GPU capture (call when app enters background).
    /// The capture thread stays alive but skips Metal rendering.
    /// Network transports remain active so clients stay connected.
    public func suspend() {
        lock.lock()
        suspended = true
        lock.unlock()
        logger.info("Video streamer suspended (app backgrounded)")
    }

    /// Resume GPU capture (call when app returns to foreground).
    public func resume() {
        lock.lock()
        suspended = false
        lock.unlock()
        logger.info("Video streamer resumed (app foregrounded)")
    }

    /// Measured output FPS (updated once per second on capture thread).
    public var measuredFPS: Double {
        fpsLock.lock()
        defer { fpsLock.unlock() }
        return _measuredFPS
    }

    /// Whether a receiver is connected.
    public var hasReceiver: Bool {
        switch config.transportMode {
        case .rawUDP:
            return udpTransport?.HasReceiver() ?? false
        case .rtpRTSP:
            return rtpTransport?.HasReceiver() ?? false
        case .mjpegHTTP:
            return mjpegServer?.HasReceiver() ?? false
        }
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
            let isSuspended = suspended
            lock.unlock()
            guard shouldRun else { break }

            if isSuspended {
                // App is in background — skip GPU work to avoid
                // kIOGPUCommandBufferCallbackErrorBackgroundExecutionNotPermitted
                Thread.sleep(forTimeInterval: 0.5)
                continue
            }

            let frameStart = CACurrentMediaTime()
            // Wrap in autoreleasepool — CGImage/CGDataProvider/NSMutableData
            // from the JPEG encoder create autoreleased objects that won't be
            // drained automatically on this background thread.
            autoreleasepool {
                captureFrame()
            }
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

        // Send via active transport
        encodedData.withUnsafeBytes { ptr in
            let dataPtr = ptr.baseAddress!.assumingMemoryBound(to: UInt8.self)
            let count = encodedData.count
            switch config.transportMode {
            case .rawUDP:
                _ = udpTransport?.SendFrame(desc, dataPtr, count)
            case .rtpRTSP:
                _ = rtpTransport?.SendFrame(desc, dataPtr, count)
            case .mjpegHTTP:
                _ = mjpegServer?.SendJPEG(dataPtr, count)
            }
        }

        // Update FPS measurement (once per second)
        fpsFrameCount += 1
        let now = CACurrentMediaTime()
        let elapsed = now - fpsLastUpdate
        if elapsed >= 1.0 {
            let fps = Double(fpsFrameCount) / elapsed
            fpsLock.lock()
            _measuredFPS = fps
            fpsLock.unlock()
            fpsFrameCount = 0
            fpsLastUpdate = now
        }
    }

}
