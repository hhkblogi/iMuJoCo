// mjc_video_quic_transport.swift
// HEVC-over-QUIC video transport using Network.framework.
//
// Architecture:
//   NWListener (QUIC, ALPN "imujoco-video")
//     → newConnectionGroupHandler → NWConnectionGroup (QUIC tunnel)
//       → group.newConnectionHandler → NWConnection (bidirectional stream)
//         → client sends "HELO" → server sends metadata + frames
//
// Wire format (on bidirectional QUIC stream):
//   Client → Server: [4-byte "HELO"]
//   Server → Client: [12-byte metadata]                  (once)
//   Server → Client: [4-byte size LE][16-byte hdr][HEVC] (repeated)

import Foundation
import Network
import os.log

private let logger = Logger(subsystem: "com.mujoco.video", category: "QUICTransport")

// MARK: - Wire Protocol

/// Per-frame header sent after the 4-byte length prefix.
/// 16 bytes, little-endian.
struct MJQUICFrameHeader {
    var frameNumber: UInt64
    var simulationTime: Double

    func serialize() -> Data {
        var data = Data(capacity: 16)
        var fn = frameNumber.littleEndian
        var st = simulationTime.bitPattern.littleEndian
        withUnsafeBytes(of: &fn) { data.append(contentsOf: $0) }
        withUnsafeBytes(of: &st) { data.append(contentsOf: $0) }
        return data
    }
}

// MARK: - QUIC Transport Server

public final class MJVideoQUICTransport {

    private let queue = DispatchQueue(label: "com.mujoco.video.quic", qos: .userInitiated)

    private var listener: NWListener?

    // Connected streams (bidirectional QUIC streams from clients)
    private var streams: [NWConnection] = []
    private let streamsLock = NSLock()

    // Per-stream state
    private var sendPending: Set<ObjectIdentifier> = []
    private var metadataSent: Set<ObjectIdentifier> = []
    private var readyStreams: Set<ObjectIdentifier> = []  // Streams that sent HELO

    // Connection metadata
    private var width: UInt32 = 256
    private var height: UInt32 = 256

    private var _isActive = false

    // Keep groups alive
    private var groups: [NWConnectionGroup] = []

    // Signaled when listener reaches .cancelled state
    private let cancelSemaphore = DispatchSemaphore(value: 0)

    // MARK: - Public API

    public init() {}

    deinit {
        stop()
    }

    public func start(port: UInt16, width: Int = 256, height: Int = 256) -> Bool {
        guard !_isActive else { return true }

        self.width = UInt32(width)
        self.height = UInt32(height)

        guard let quicOptions = MJQUICIdentity.quicOptions() else {
            logger.error("Cannot start QUIC transport: no TLS identity")
            return false
        }

        let params = NWParameters(quic: quicOptions)
        // Restrict to WiFi/Ethernet only — no cellular or other interfaces
        params.prohibitedInterfaceTypes = [.cellular, .other]

        do {
            listener = try NWListener(using: params, on: NWEndpoint.Port(rawValue: port)!)
        } catch {
            logger.error("Failed to create NWListener: \(error.localizedDescription)")
            return false
        }

        guard let listener else { return false }

        listener.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready:
                if let port = self?.listener?.port {
                    logger.info("QUIC listener ready on port \(port.rawValue)")
                }
            case .failed(let error):
                logger.error("QUIC listener failed: \(error.localizedDescription)")
                self?.stop()
            case .cancelled:
                logger.info("QUIC listener cancelled")
                self?.cancelSemaphore.signal()
            default:
                break
            }
        }

        // Use newConnectionGroupHandler for QUIC tunnels (per WWDC 2021)
        listener.newConnectionGroupHandler = { [weak self] group in
            self?.handleNewGroup(group)
        }

        listener.start(queue: queue)
        _isActive = true
        logger.info("QUIC transport started on port \(port) [\(width)x\(height)]")
        return true
    }

    public func stop() {
        guard _isActive else { return }
        _isActive = false

        // 1) Force-cancel all streams (immediately disconnects remote clients).
        //    This releases the QUIC connections that hold the UDP socket open.
        streamsLock.lock()
        let currentStreams = streams
        let currentGroups = groups
        streams.removeAll()
        sendPending.removeAll()
        metadataSent.removeAll()
        readyStreams.removeAll()
        groups.removeAll()
        streamsLock.unlock()

        for stream in currentStreams {
            stream.stateUpdateHandler = nil
            stream.forceCancel()
        }
        for group in currentGroups {
            group.stateUpdateHandler = nil
            group.cancel()
        }

        // 2) Cancel listener (releases the UDP port binding).
        let hadListener = listener != nil
        listener?.cancel()

        // 3) Wait for the listener to fully release the port.
        if hadListener {
            _ = cancelSemaphore.wait(timeout: .now() + 3.0)
        }
        listener = nil

        logger.info("QUIC transport stopped")
    }

    public var isActive: Bool { _isActive }

    public var hasReceiver: Bool {
        streamsLock.lock()
        defer { streamsLock.unlock() }
        return !readyStreams.isEmpty
    }

    /// Send an HEVC frame to all ready clients.
    public func sendFrame(frameNumber: UInt64, simulationTime: Double, hevcData: Data) {
        guard _isActive else { return }

        let header = MJQUICFrameHeader(
            frameNumber: frameNumber,
            simulationTime: simulationTime
        )
        let headerData = header.serialize()

        // Build length-prefixed frame: [4-byte size][16-byte header][HEVC data]
        let framePayloadSize = UInt32(headerData.count + hevcData.count)
        var frameData = Data(capacity: 4 + Int(framePayloadSize))
        var sizeLE = framePayloadSize.littleEndian
        withUnsafeBytes(of: &sizeLE) { frameData.append(contentsOf: $0) }
        frameData.append(headerData)
        frameData.append(hevcData)

        streamsLock.lock()
        let currentStreams = streams
        streamsLock.unlock()

        for stream in currentStreams {
            let streamId = ObjectIdentifier(stream)

            streamsLock.lock()
            let isReady = readyStreams.contains(streamId)
            let needsMetadata = !metadataSent.contains(streamId)
            let isPending = sendPending.contains(streamId)
            if isReady && !isPending {
                sendPending.insert(streamId)
                if needsMetadata {
                    metadataSent.insert(streamId)
                }
            }
            streamsLock.unlock()

            // Only send to streams that have completed HELO handshake
            guard isReady, !isPending else { continue }

            if needsMetadata {
                var fullData = Data(capacity: 12 + frameData.count)
                var w = width.littleEndian
                var h = height.littleEndian
                var fmt: UInt32 = 4
                fmt = fmt.littleEndian
                withUnsafeBytes(of: &w) { fullData.append(contentsOf: $0) }
                withUnsafeBytes(of: &h) { fullData.append(contentsOf: $0) }
                withUnsafeBytes(of: &fmt) { fullData.append(contentsOf: $0) }
                fullData.append(frameData)

                stream.send(
                    content: fullData,
                    contentContext: .defaultMessage,
                    isComplete: false,
                    completion: .contentProcessed { [weak self] error in
                        self?.streamsLock.lock()
                        self?.sendPending.remove(streamId)
                        self?.streamsLock.unlock()
                        if let error {
                            logger.debug("QUIC send error: \(error.localizedDescription)")
                        } else {
                            logger.info("Sent metadata + first frame (\(fullData.count) bytes)")
                        }
                    }
                )
            } else {
                stream.send(
                    content: frameData,
                    contentContext: .defaultMessage,
                    isComplete: false,
                    completion: .contentProcessed { [weak self] error in
                        self?.streamsLock.lock()
                        self?.sendPending.remove(streamId)
                        self?.streamsLock.unlock()
                        if let error {
                            logger.debug("QUIC send error: \(error.localizedDescription)")
                        }
                    }
                )
            }
        }
    }

    // MARK: - QUIC Tunnel (Connection Group)

    private func handleNewGroup(_ group: NWConnectionGroup) {
        logger.info("New QUIC tunnel")

        group.stateUpdateHandler = { state in
            switch state {
            case .ready:
                logger.info("QUIC tunnel ready")
            case .failed(let error):
                logger.error("QUIC tunnel failed: \(error.localizedDescription)")
            case .cancelled:
                logger.info("QUIC tunnel cancelled")
            default:
                break
            }
        }

        // Receive client-initiated streams on this tunnel
        group.newConnectionHandler = { [weak self] stream in
            self?.handleNewStream(stream)
        }

        streamsLock.lock()
        groups.append(group)
        streamsLock.unlock()

        group.start(queue: queue)
    }

    // MARK: - Stream Handling

    private func handleNewStream(_ stream: NWConnection) {
        stream.stateUpdateHandler = { [weak self, weak stream] state in
            guard let self, let stream else { return }
            switch state {
            case .ready:
                logger.info("QUIC stream ready: \(String(describing: stream.endpoint))")
                // Wait for client HELO before sending
                self.waitForHello(stream)
            case .failed(let error):
                logger.info("QUIC stream failed: \(error.localizedDescription)")
                self.removeStream(stream)
            case .cancelled:
                logger.info("QUIC stream cancelled")
                self.removeStream(stream)
            default:
                break
            }
        }

        streamsLock.lock()
        streams.append(stream)
        streamsLock.unlock()

        stream.start(queue: queue)
    }

    /// Wait for 4-byte "HELO" from client, then mark stream as ready for sending.
    private func waitForHello(_ stream: NWConnection) {
        stream.receive(minimumIncompleteLength: 4, maximumLength: 64) { [weak self] data, _, _, error in
            guard let self else { return }

            if let error {
                logger.error("Failed to receive HELO: \(error.localizedDescription)")
                self.removeStream(stream)
                return
            }

            if let data {
                let msg = String(data: data.prefix(4), encoding: .utf8) ?? "?"
                logger.info("Received from client: \(msg) (\(data.count) bytes)")
            }

            let streamId = ObjectIdentifier(stream)
            self.streamsLock.lock()
            self.readyStreams.insert(streamId)
            self.streamsLock.unlock()
            logger.info("Stream marked ready for sending")

            // Continue receiving from client (disconnect detection)
            self.receiveFromStream(stream)
        }
    }

    private func receiveFromStream(_ stream: NWConnection) {
        stream.receive(minimumIncompleteLength: 1, maximumLength: 1024) { [weak self] _, _, isComplete, error in
            if isComplete || error != nil {
                self?.removeStream(stream)
                return
            }
            self?.receiveFromStream(stream)
        }
    }

    private func removeStream(_ stream: NWConnection) {
        let streamId = ObjectIdentifier(stream)
        streamsLock.lock()
        let wasTracked = streams.contains { $0 === stream }
        streams.removeAll { $0 === stream }
        sendPending.remove(streamId)
        metadataSent.remove(streamId)
        readyStreams.remove(streamId)
        streamsLock.unlock()
        // Only cancel if still tracked (stop() already force-cancelled all)
        if wasTracked { stream.cancel() }
    }
}
