// HevcVideoClient.swift
// HEVC/RTP video receiver client using Network.framework.
// Connects to the iMuJoCo RTSP server, negotiates RTP transport,
// receives HEVC frames via RTP (RFC 7798), decodes via VideoToolbox
// hardware decoder, and publishes CGImage frames.
//
// RTSP negotiation (TCP):
//   Client → Server: OPTIONS / DESCRIBE / SETUP / PLAY / TEARDOWN
// RTP data (UDP):
//   Server → Client: RTP packets with HEVC NAL payloads (single or FU)

import CoreGraphics
import CoreImage
import CoreVideo
import Foundation
import Network
import VideoToolbox
import os.log

private let logger = Logger(subsystem: "com.imujoco.hevc-receiver", category: "HevcVideoClient")

// MARK: - Latency Stats

struct LatencyStats {
    var frameNumber: UInt64 = 0
    var simulationTime: Double = 0
    var decodeLatencyMs: Double = 0
    var receiveTimestamp: Double = 0
    var framesReceived: UInt64 = 0
    var framesDecoded: UInt64 = 0
    var framesDropped: UInt64 = 0
}

// MARK: - HEVC Video Client

@Observable
final class HevcVideoClient {
    // Published state
    private(set) var currentFrame: CGImage?
    private(set) var stats = LatencyStats()
    private(set) var isConnecting = false
    private(set) var isConnected = false
    private(set) var connectionStatus: String = "Disconnected"

    // RTSP state
    private var rtspConnection: NWConnection?
    private var rtspSessionId: String?
    private var cseq: Int = 0
    private let queue = DispatchQueue(label: "com.imujoco.hevc-receiver.network", qos: .userInitiated)

    // RTP state (outgoing NWConnection to server's RTP/UDP port)
    private var rtpConnection: NWConnection?
    private var rtpPort: UInt16 = 0

    // RTP depacketization — pre-allocated buffers
    private var nalAssemblyBuffer = Data(capacity: 256 * 1024)
    private var frameBuffer = Data(capacity: 256 * 1024)
    private var currentTimestamp: UInt32 = 0
    private var assemblingFU = false

    // Video decode
    private var decompressionSession: VTDecompressionSession?
    private var formatDescription: CMVideoFormatDescription?
    private let ciContext = CIContext(options: [.useSoftwareRenderer: false])

    // HEVC parameter sets
    private var vps: Data?
    private var sps: Data?
    private var pps: Data?

    // MARK: - Connect

    func connect(host: String, port: UInt16) {
        disconnect()

        isConnecting = true
        connectionStatus = "Connecting to \(host):\(port)..."

        // Create outgoing UDP connection to the server's RTP port.
        // RTP and RTSP share the same port number (UDP vs TCP).
        // Using NWConnection (outgoing) avoids macOS firewall blocking
        // incoming UDP that NWListener would require.
        let endpoint = NWEndpoint.hostPort(
            host: NWEndpoint.Host(host),
            port: NWEndpoint.Port(rawValue: port)!
        )
        let rtpConn = NWConnection(to: endpoint, using: .udp)
        self.rtpConnection = rtpConn

        rtpConn.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready:
                // Send a register byte to establish the bidirectional UDP flow
                // and trigger local port assignment.
                rtpConn.send(content: Data([0x00]), completion: .contentProcessed { [weak self] _ in
                    guard let self else { return }

                    // Extract OS-assigned local port
                    if let localEndpoint = rtpConn.currentPath?.localEndpoint,
                       case .hostPort(_, let p) = localEndpoint {
                        self.rtpPort = p.rawValue
                    }
                    logger.info("RTP connection ready, local port \(self.rtpPort)")

                    // Start receiving RTP packets
                    self.receiveRTPLoop(rtpConn)

                    // Start RTSP negotiation
                    self.startRTSP(host: host, port: port)
                })

            case .failed(let error):
                logger.error("RTP connection failed: \(error.localizedDescription)")
                DispatchQueue.main.async {
                    self.connectionStatus = "RTP connection failed"
                }
            default:
                break
            }
        }

        rtpConn.start(queue: queue)
    }

    func disconnect() {
        // Send TEARDOWN if we have a session
        if let conn = rtspConnection, let sessionId = rtspSessionId {
            sendRTSPTeardown(conn: conn, sessionId: sessionId)
        }

        rtspTimeoutWork?.cancel()
        rtspTimeoutWork = nil
        rtspConnection?.cancel()
        rtspConnection = nil
        rtspSessionId = nil
        cseq = 0

        rtpConnection?.cancel()
        rtpConnection = nil
        rtpPort = 0

        decompressionSession = nil
        formatDescription = nil
        vps = nil
        sps = nil
        pps = nil

        nalAssemblyBuffer.removeAll(keepingCapacity: true)
        frameBuffer.removeAll(keepingCapacity: true)
        currentTimestamp = 0
        assemblingFU = false

        isConnecting = false
        isConnected = false
        connectionStatus = "Disconnected"
    }

    // MARK: - RTP Receive Loop

    private func receiveRTPLoop(_ conn: NWConnection) {
        conn.receiveMessage { [weak self] data, _, _, error in
            guard let self else { return }

            if let error {
                logger.debug("RTP receive error: \(error.localizedDescription)")
                return
            }

            if let data, data.count >= 12 {
                self.processRTPPacket(data)
            }

            // Continue receiving (isComplete is always true for UDP datagrams)
            if self.rtpConnection != nil {
                self.receiveRTPLoop(conn)
            }
        }
    }

    // MARK: - RTP Depacketization (RFC 7798)

    private func processRTPPacket(_ packet: Data) {
        // Parse 12-byte RTP header by offset (zero-copy)
        guard packet.count >= 12 else { return }

        let byte0 = packet.withUnsafeBytes { $0.load(fromByteOffset: 0, as: UInt8.self) }
        let version = (byte0 >> 6) & 0x03
        guard version == 2 else { return }

        let byte1 = packet.withUnsafeBytes { $0.load(fromByteOffset: 1, as: UInt8.self) }
        let marker = (byte1 & 0x80) != 0

        let timestamp = packet.withUnsafeBytes {
            CFSwapInt32BigToHost($0.loadUnaligned(fromByteOffset: 4, as: UInt32.self))
        }

        let payloadOffset = 12
        let payloadLength = packet.count - payloadOffset
        guard payloadLength >= 2 else { return }

        // If timestamp changed, we have a new frame — flush previous
        if timestamp != currentTimestamp && !frameBuffer.isEmpty {
            decodeFrame(timestamp: currentTimestamp)
            frameBuffer.removeAll(keepingCapacity: true)
            assemblingFU = false
        }
        currentTimestamp = timestamp

        // Read HEVC NAL header (2 bytes) from payload
        let nalByte0 = packet.withUnsafeBytes { $0.load(fromByteOffset: payloadOffset, as: UInt8.self) }
        let nalType = (nalByte0 >> 1) & 0x3F

        if nalType == 49 {
            // Fragmentation Unit (FU) — RFC 7798 §4.4.3
            guard payloadLength >= 3 else { return }

            let fuHeader = packet.withUnsafeBytes { $0.load(fromByteOffset: payloadOffset + 2, as: UInt8.self) }
            let startBit = (fuHeader & 0x80) != 0
            let endBit = (fuHeader & 0x40) != 0
            let fuType = fuHeader & 0x3F

            let fuPayloadOffset = payloadOffset + 3
            let fuPayloadLength = packet.count - fuPayloadOffset
            guard fuPayloadLength > 0 else { return }

            if startBit {
                // Start fragment — reconstruct 2-byte NAL header
                nalAssemblyBuffer.removeAll(keepingCapacity: true)

                let nalByte1 = packet.withUnsafeBytes { $0.load(fromByteOffset: payloadOffset + 1, as: UInt8.self) }
                // Reconstruct: forbidden(1) | fuType(6) | layerID_hi(1), layerID_lo(5) | tid(3)
                let reconstructedByte0 = (nalByte0 & 0x81) | (fuType << 1)
                nalAssemblyBuffer.append(reconstructedByte0)
                nalAssemblyBuffer.append(nalByte1)

                // Append FU payload
                packet.withUnsafeBytes { rawBuf in
                    nalAssemblyBuffer.append(rawBuf.baseAddress!.advanced(by: fuPayloadOffset).assumingMemoryBound(to: UInt8.self), count: fuPayloadLength)
                }
                assemblingFU = true

            } else if assemblingFU {
                // Middle or end fragment — append FU payload
                packet.withUnsafeBytes { rawBuf in
                    nalAssemblyBuffer.append(rawBuf.baseAddress!.advanced(by: fuPayloadOffset).assumingMemoryBound(to: UInt8.self), count: fuPayloadLength)
                }

                if endBit {
                    // End fragment — flush assembled NAL to frameBuffer
                    appendNALToFrameBuffer(nalAssemblyBuffer)
                    nalAssemblyBuffer.removeAll(keepingCapacity: true)
                    assemblingFU = false
                }
            }

        } else if nalType >= 1 && nalType <= 47 {
            // Single NAL Unit Packet — entire NAL fits in one RTP packet
            // Payload is the raw NAL unit (including 2-byte header)
            let nalData = packet.subdata(in: payloadOffset..<packet.count)
            appendNALToFrameBuffer(nalData)
        }

        // Frame boundary: marker bit means all NALs for this timestamp received
        if marker && !frameBuffer.isEmpty {
            decodeFrame(timestamp: timestamp)
            frameBuffer.removeAll(keepingCapacity: true)
            assemblingFU = false
        }
    }

    /// Append a NAL unit to frameBuffer in HVCC format (4-byte BE length prefix + NAL data)
    private func appendNALToFrameBuffer(_ nalData: Data) {
        var lengthBE = CFSwapInt32HostToBig(UInt32(nalData.count))
        withUnsafeBytes(of: &lengthBE) { frameBuffer.append(contentsOf: $0) }
        frameBuffer.append(nalData)
    }

    // MARK: - HEVC Decode

    /// Decode a complete frame from frameBuffer (HVCC format: 4-byte length + NAL data).
    /// Mirrors the QUIC receiver's decodeHEVC — offset-based NAL parsing, single CMBlockBuffer copy.
    private func decodeFrame(timestamp: UInt32) {
        let decodeStart = ProcessInfo.processInfo.systemUptime
        let receiveTime = decodeStart

        // Parse HVCC NAL units from frameBuffer to extract VPS/SPS/PPS
        var nalOffset = 0
        let bufferSize = frameBuffer.count

        while nalOffset + 4 < bufferSize {
            let nalSize = frameBuffer.withUnsafeBytes {
                CFSwapInt32BigToHost($0.loadUnaligned(fromByteOffset: nalOffset, as: UInt32.self))
            }

            guard nalSize > 0, nalOffset + 4 + Int(nalSize) <= bufferSize else { break }

            let nalDataOffset = nalOffset + 4

            // Read NAL type byte directly — no copy needed
            let nalTypeByte = frameBuffer.withUnsafeBytes {
                $0.load(fromByteOffset: nalDataOffset, as: UInt8.self)
            }
            let nalType = (nalTypeByte >> 1) & 0x3F

            // Only copy VPS/SPS/PPS (small, extracted once)
            switch nalType {
            case 32: vps = frameBuffer.subdata(in: nalDataOffset..<nalDataOffset + Int(nalSize))
            case 33: sps = frameBuffer.subdata(in: nalDataOffset..<nalDataOffset + Int(nalSize))
            case 34: pps = frameBuffer.subdata(in: nalDataOffset..<nalDataOffset + Int(nalSize))
            default: break
            }

            nalOffset += 4 + Int(nalSize)
        }

        // Create format description if we have parameter sets
        if let vps, let sps, let pps, formatDescription == nil {
            createFormatDescription(vps: vps, sps: sps, pps: pps)
        }

        guard let formatDesc = formatDescription else {
            logger.debug("Waiting for parameter sets (VPS/SPS/PPS)")
            return
        }

        let hevcLength = frameBuffer.count

        // Create CMBlockBuffer: single alloc, copy directly from frameBuffer
        var blockBuffer: CMBlockBuffer?
        var status = CMBlockBufferCreateWithMemoryBlock(
            allocator: kCFAllocatorDefault,
            memoryBlock: nil,
            blockLength: hevcLength,
            blockAllocator: nil,
            customBlockSource: nil,
            offsetToData: 0,
            dataLength: hevcLength,
            flags: 0,
            blockBufferOut: &blockBuffer
        )

        guard status == kCMBlockBufferNoErr, let blockBuf = blockBuffer else {
            logger.error("Failed to create CMBlockBuffer: \(status)")
            return
        }

        // Single copy: frameBuffer → CMBlockBuffer
        status = frameBuffer.withUnsafeBytes { rawPtr in
            CMBlockBufferReplaceDataBytes(
                with: rawPtr.baseAddress!,
                blockBuffer: blockBuf,
                offsetIntoDestination: 0,
                dataLength: hevcLength
            )
        }

        guard status == kCMBlockBufferNoErr else {
            logger.error("Failed to copy data into CMBlockBuffer: \(status)")
            return
        }

        // Derive frame number and simulation time from RTP timestamp (90kHz clock)
        let simulationTime = Double(timestamp) / 90000.0
        let frameNumber = stats.framesReceived

        var sampleBuffer: CMSampleBuffer?
        var timingInfo = CMSampleTimingInfo(
            duration: CMTime(value: 1, timescale: 30),
            presentationTimeStamp: CMTime(value: CMTimeValue(timestamp), timescale: 90000),
            decodeTimeStamp: .invalid
        )

        let sampleStatus = CMSampleBufferCreateReady(
            allocator: kCFAllocatorDefault,
            dataBuffer: blockBuf,
            formatDescription: formatDesc,
            sampleCount: 1,
            sampleTimingEntryCount: 1,
            sampleTimingArray: &timingInfo,
            sampleSizeEntryCount: 1,
            sampleSizeArray: [hevcLength],
            sampleBufferOut: &sampleBuffer
        )

        guard sampleStatus == noErr, let sampleBuffer else {
            logger.error("Failed to create CMSampleBuffer: \(sampleStatus)")
            return
        }

        if decompressionSession == nil {
            createDecompressionSession(formatDescription: formatDesc)
        }

        guard let session = decompressionSession else { return }

        var flagsOut = VTDecodeInfoFlags()
        let decodeStatus = VTDecompressionSessionDecodeFrame(
            session,
            sampleBuffer: sampleBuffer,
            flags: [._EnableAsynchronousDecompression],
            infoFlagsOut: &flagsOut
        ) { [weak self] (status: OSStatus, _: VTDecodeInfoFlags, imageBuffer: CVImageBuffer?, _: [CMTaggedBuffer]?, _: CMTime, _: CMTime) in
            guard let self else { return }
            if status != noErr {
                logger.debug("Decode callback error: \(status)")
                DispatchQueue.main.async { self.stats.framesDropped += 1 }
                return
            }
            guard let pixelBuffer = imageBuffer else { return }
            self.handleDecodedFrame(pixelBuffer)
        }

        if decodeStatus != noErr {
            logger.error("VTDecompressionSessionDecodeFrame failed: \(decodeStatus)")
            DispatchQueue.main.async { self.stats.framesDropped += 1 }
            return
        }

        VTDecompressionSessionWaitForAsynchronousFrames(session)

        // Batched stats update — single dispatch
        let decodeMs = (ProcessInfo.processInfo.systemUptime - decodeStart) * 1000
        DispatchQueue.main.async {
            self.stats.framesReceived += 1
            self.stats.frameNumber = frameNumber + 1
            self.stats.simulationTime = simulationTime
            self.stats.receiveTimestamp = receiveTime
            self.stats.decodeLatencyMs = decodeMs
            self.stats.framesDecoded += 1
        }
    }

    private func createFormatDescription(vps: Data, sps: Data, pps: Data) {
        let parameterSets: [Data] = [vps, sps, pps]
        let sizes = parameterSets.map { $0.count }

        var formatDesc: CMVideoFormatDescription?

        let status = parameterSets.withUnsafeBufferPointers { ptrs in
            CMVideoFormatDescriptionCreateFromHEVCParameterSets(
                allocator: kCFAllocatorDefault,
                parameterSetCount: 3,
                parameterSetPointers: ptrs,
                parameterSetSizes: sizes,
                nalUnitHeaderLength: 4,
                extensions: nil,
                formatDescriptionOut: &formatDesc
            )
        }

        if status == noErr, let formatDesc {
            self.formatDescription = formatDesc
            logger.info("Created HEVC format description")
        } else {
            logger.error("Failed to create HEVC format description: \(status)")
        }
    }

    private func createDecompressionSession(formatDescription: CMVideoFormatDescription) {
        let attributes: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
        ]

        var session: VTDecompressionSession?
        let status = VTDecompressionSessionCreate(
            allocator: kCFAllocatorDefault,
            formatDescription: formatDescription,
            decoderSpecification: nil,
            imageBufferAttributes: attributes as CFDictionary,
            outputCallback: nil,
            decompressionSessionOut: &session
        )

        if status == noErr, let session {
            self.decompressionSession = session
            logger.info("Created VTDecompressionSession")
        } else {
            logger.error("Failed to create VTDecompressionSession: \(status)")
        }
    }

    /// Convert decoded pixel buffer to CGImage via GPU-accelerated CIContext.
    private func handleDecodedFrame(_ pixelBuffer: CVPixelBuffer) {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        guard let cgImage = ciContext.createCGImage(ciImage, from: ciImage.extent) else { return }
        DispatchQueue.main.async {
            self.currentFrame = cgImage
        }
    }

    // MARK: - RTSP Negotiation (TCP) with Auto-Retry

    private static let rtspRetryInterval: TimeInterval = 2.0
    private static let rtspConnectTimeout: TimeInterval = 3.0
    private var rtspTimeoutWork: DispatchWorkItem?

    private func startRTSP(host: String, port: UInt16) {
        // Clean up previous RTSP connection if retrying
        rtspTimeoutWork?.cancel()
        rtspTimeoutWork = nil
        rtspConnection?.cancel()
        rtspConnection = nil
        rtspSessionId = nil
        cseq = 0

        // Don't retry if user cancelled
        guard isConnecting else { return }

        DispatchQueue.main.async {
            self.connectionStatus = "Connecting to \(host):\(port)..."
        }

        let endpoint = NWEndpoint.hostPort(
            host: NWEndpoint.Host(host),
            port: NWEndpoint.Port(rawValue: port)!
        )

        let conn = NWConnection(to: endpoint, using: .tcp)
        rtspConnection = conn

        // Timeout: if TCP doesn't connect within a few seconds, cancel and retry
        let timeout = DispatchWorkItem { [weak self] in
            guard let self, self.isConnecting, self.rtspConnection === conn else { return }
            conn.cancel()
            self.retryRTSP(host: host, port: port, reason: "Server unreachable")
        }
        rtspTimeoutWork = timeout
        queue.asyncAfter(deadline: .now() + Self.rtspConnectTimeout, execute: timeout)

        conn.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready:
                // Connected — cancel timeout
                self.rtspTimeoutWork?.cancel()
                self.rtspTimeoutWork = nil
                logger.info("RTSP TCP connection ready")
                self.sendRTSPOptions(conn: conn, host: host, port: port)
            case .failed:
                self.rtspTimeoutWork?.cancel()
                self.rtspTimeoutWork = nil
                self.retryRTSP(host: host, port: port, reason: "Connection failed")
            case .cancelled:
                break
            default:
                break
            }
        }

        conn.start(queue: queue)
    }

    private func retryRTSP(host: String, port: UInt16, reason: String) {
        guard isConnecting else { return }
        let delay = Self.rtspRetryInterval
        logger.info("RTSP: \(reason), retrying in \(delay)s...")
        DispatchQueue.main.async {
            self.connectionStatus = "\(reason) — retrying..."
        }
        queue.asyncAfter(deadline: .now() + delay) { [weak self] in
            self?.startRTSP(host: host, port: port)
        }
    }

    // MARK: - RTSP Request Helpers

    private func nextCSeq() -> Int {
        cseq += 1
        return cseq
    }

    private func sendRTSPRequest(_ conn: NWConnection, method: String, uri: String,
                                  extraHeaders: String = "", completion: @escaping (String?) -> Void) {
        let seq = nextCSeq()
        let request = "\(method) \(uri) RTSP/1.0\r\nCSeq: \(seq)\r\n\(extraHeaders)\r\n"

        conn.send(content: Data(request.utf8), completion: .contentProcessed { error in
            if let error {
                logger.error("Failed to send \(method): \(error.localizedDescription)")
                completion(nil)
                return
            }
        })

        // Receive response
        conn.receive(minimumIncompleteLength: 1, maximumLength: 8192) { data, _, _, error in
            if let error {
                logger.error("Failed to receive \(method) response: \(error.localizedDescription)")
                completion(nil)
                return
            }
            guard let data, let response = String(data: data, encoding: .utf8) else {
                completion(nil)
                return
            }
            completion(response)
        }
    }

    private func parseRTSPStatusCode(_ response: String) -> Int? {
        // First line: "RTSP/1.0 200 OK"
        guard let firstLine = response.split(separator: "\r\n", maxSplits: 1).first else { return nil }
        let parts = firstLine.split(separator: " ", maxSplits: 2)
        guard parts.count >= 2, let code = Int(parts[1]) else { return nil }
        return code
    }

    private func parseRTSPHeader(_ response: String, name: String) -> String? {
        let lines = response.split(separator: "\r\n")
        let prefix = name + ": "
        for line in lines {
            if line.hasPrefix(prefix) {
                return String(line.dropFirst(prefix.count))
            }
        }
        return nil
    }

    // MARK: - RTSP Sequence: OPTIONS → DESCRIBE → SETUP → PLAY

    private func sendRTSPOptions(conn: NWConnection, host: String, port: UInt16) {
        let uri = "rtsp://\(host):\(port)/camera0"

        sendRTSPRequest(conn, method: "OPTIONS", uri: uri) { [weak self] response in
            guard let self, let response else {
                self?.failRTSP("OPTIONS failed", host: host, port: port)
                return
            }
            guard let status = self.parseRTSPStatusCode(response), status == 200 else {
                self.failRTSP("OPTIONS rejected", host: host, port: port)
                return
            }
            logger.info("RTSP OPTIONS OK")
            self.sendRTSPDescribe(conn: conn, host: host, port: port, uri: uri)
        }
    }

    private func sendRTSPDescribe(conn: NWConnection, host: String, port: UInt16, uri: String) {
        sendRTSPRequest(conn, method: "DESCRIBE", uri: uri,
                        extraHeaders: "Accept: application/sdp\r\n") { [weak self] response in
            guard let self, let response else {
                self?.failRTSP("DESCRIBE failed", host: host, port: port)
                return
            }
            guard let status = self.parseRTSPStatusCode(response), status == 200 else {
                self.failRTSP("DESCRIBE rejected", host: host, port: port)
                return
            }
            logger.info("RTSP DESCRIBE OK")
            // SDP parsed but we don't strictly need it — we know the format
            self.sendRTSPSetup(conn: conn, host: host, port: port, uri: uri)
        }
    }

    private func sendRTSPSetup(conn: NWConnection, host: String, port: UInt16, uri: String) {
        let transport = "Transport: RTP/AVP;unicast;client_port=\(rtpPort)-\(rtpPort + 1)\r\n"
        logger.info("RTSP SETUP: client_port=\(self.rtpPort)-\(self.rtpPort + 1)")

        sendRTSPRequest(conn, method: "SETUP", uri: uri,
                        extraHeaders: transport) { [weak self] response in
            guard let self, let response else {
                self?.failRTSP("SETUP failed", host: host, port: port)
                return
            }
            guard let status = self.parseRTSPStatusCode(response), status == 200 else {
                self.failRTSP("SETUP rejected", host: host, port: port)
                return
            }

            // Extract Session header
            if let sessionHeader = self.parseRTSPHeader(response, name: "Session") {
                // Strip timeout suffix if present
                let sessionId = sessionHeader.split(separator: ";").first.map(String.init) ?? sessionHeader
                self.rtspSessionId = sessionId
            }

            logger.info("RTSP SETUP OK, session=\(self.rtspSessionId ?? "?")")
            self.sendRTSPPlay(conn: conn, host: host, port: port, uri: uri)
        }
    }

    private func sendRTSPPlay(conn: NWConnection, host: String, port: UInt16, uri: String) {
        let sessionHeader = rtspSessionId.map { "Session: \($0)\r\n" } ?? ""

        sendRTSPRequest(conn, method: "PLAY", uri: uri,
                        extraHeaders: sessionHeader) { [weak self] response in
            guard let self, let response else {
                self?.failRTSP("PLAY failed", host: host, port: port)
                return
            }
            guard let status = self.parseRTSPStatusCode(response), status == 200 else {
                self.failRTSP("PLAY rejected", host: host, port: port)
                return
            }

            logger.info("RTSP PLAY OK — streaming started")
            DispatchQueue.main.async {
                self.isConnecting = false
                self.isConnected = true
                self.connectionStatus = "Streaming (RTP)"
            }
        }
    }

    private func sendRTSPTeardown(conn: NWConnection, sessionId: String) {
        // Best-effort teardown, don't wait for response
        let seq = nextCSeq()
        let request = "TEARDOWN rtsp://localhost/camera0 RTSP/1.0\r\nCSeq: \(seq)\r\nSession: \(sessionId)\r\n\r\n"
        conn.send(content: Data(request.utf8), completion: .contentProcessed { _ in })
    }

    private func failRTSP(_ message: String, host: String, port: UInt16) {
        retryRTSP(host: host, port: port, reason: message)
    }
}

// MARK: - Helper: withUnsafeBufferPointers for Array of Data

private extension Array where Element == Data {
    func withUnsafeBufferPointers<R>(_ body: (UnsafePointer<UnsafePointer<UInt8>>) -> R) -> R {
        let ptrs = UnsafeMutablePointer<UnsafePointer<UInt8>>.allocate(capacity: count)
        defer { ptrs.deallocate() }
        return withNestedUnsafeBytes(index: 0, ptrs: ptrs, body: body)
    }

    private func withNestedUnsafeBytes<R>(
        index: Int,
        ptrs: UnsafeMutablePointer<UnsafePointer<UInt8>>,
        body: (UnsafePointer<UnsafePointer<UInt8>>) -> R
    ) -> R {
        if index >= count {
            return body(UnsafePointer(ptrs))
        }
        return self[index].withUnsafeBytes { rawPtr in
            ptrs[index] = rawPtr.baseAddress!.assumingMemoryBound(to: UInt8.self)
            return withNestedUnsafeBytes(index: index + 1, ptrs: ptrs, body: body)
        }
    }
}
