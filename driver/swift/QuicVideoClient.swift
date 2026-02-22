// QuicVideoClient.swift
// QUIC video receiver client using Network.framework.
// Connects to the iMuJoCo QUIC transport server, receives HEVC frames,
// decodes via VideoToolbox hardware decoder, and publishes CGImage frames.
//
// Wire format (on default bidirectional QUIC stream):
//   Client → Server: [4-byte "HELO"]                    -- handshake
//   Server → Client: [12-byte metadata]                  -- once
//   Server → Client: [4-byte size LE][16-byte hdr][HEVC] -- repeated

import CoreGraphics
import CoreVideo
import Foundation
import Network
import VideoToolbox
import os.log

private let logger = Logger(subsystem: "com.imujoco.quic-receiver", category: "QuicVideoClient")

// MARK: - Frame Header (matches server's MJQUICFrameHeader)

struct QUICFrameHeader {
    let frameNumber: UInt64
    let simulationTime: Double

    static let size = 16

    init?(data: Data) {
        guard data.count >= Self.size else { return nil }
        frameNumber = data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: 0, as: UInt64.self).littleEndian }
        simulationTime = data.withUnsafeBytes {
            Double(bitPattern: $0.loadUnaligned(fromByteOffset: 8, as: UInt64.self).littleEndian)
        }
    }
}

// MARK: - Connection Metadata

struct ConnectionMetadata {
    let width: UInt32
    let height: UInt32
    let format: UInt32  // 4 = HEVC

    static let size = 12

    init?(data: Data) {
        guard data.count >= Self.size else { return nil }
        width = data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: 0, as: UInt32.self).littleEndian }
        height = data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: 4, as: UInt32.self).littleEndian }
        format = data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: 8, as: UInt32.self).littleEndian }
    }
}

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

// MARK: - QUIC Video Client

@Observable
final class QuicVideoClient {
    // Published state
    private(set) var currentFrame: CGImage?
    private(set) var stats = LatencyStats()
    private(set) var isConnected = false
    private(set) var connectionStatus: String = "Disconnected"

    // Connection
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "com.imujoco.quic-receiver.network", qos: .userInitiated)
    private let decodeQueue = DispatchQueue(label: "com.imujoco.quic-receiver.decode", qos: .userInitiated)

    // Video decode
    private var decompressionSession: VTDecompressionSession?
    private var formatDescription: CMVideoFormatDescription?
    private var metadata: ConnectionMetadata?

    // HEVC parameter sets for format description
    private var vps: Data?
    private var sps: Data?
    private var pps: Data?

    // Receive buffer for accumulating partial reads
    private var receiveBuffer = Data()

    // MARK: - Connect

    func connect(host: String, port: UInt16) {
        disconnect()

        connectionStatus = "Connecting to \(host):\(port)..."

        let quicOptions = NWProtocolQUIC.Options()
        quicOptions.direction = .bidirectional

        sec_protocol_options_set_verify_block(
            quicOptions.securityProtocolOptions,
            { _, _, completionHandler in
                // Accept self-signed certificates
                completionHandler(true)
            },
            queue
        )
        sec_protocol_options_add_tls_application_protocol(
            quicOptions.securityProtocolOptions, "imujoco-video")

        let params = NWParameters(quic: quicOptions)

        let endpoint = NWEndpoint.hostPort(
            host: NWEndpoint.Host(host),
            port: NWEndpoint.Port(rawValue: port)!
        )

        let conn = NWConnection(to: endpoint, using: params)
        connection = conn

        conn.stateUpdateHandler = { [weak self] state in
            self?.queue.async {
                self?.handleStateUpdate(state)
            }
        }

        // Post the initial receive BEFORE starting the connection,
        // so we're ready to receive as soon as data arrives.
        startReceiving(conn)

        conn.start(queue: queue)
        logger.info("QUIC connection starting to \(host):\(port)")
    }

    func disconnect() {
        connection?.cancel()
        connection = nil
        decompressionSession = nil
        formatDescription = nil
        metadata = nil
        vps = nil
        sps = nil
        pps = nil
        receiveBuffer = Data()
        isConnected = false
        connectionStatus = "Disconnected"
    }

    // MARK: - State Handling

    private func handleStateUpdate(_ state: NWConnection.State) {
        switch state {
        case .ready:
            DispatchQueue.main.async {
                self.isConnected = true
                self.connectionStatus = "Connected"
            }
            logger.info("QUIC connection ready")
            // Send hello to server to trigger the bidirectional stream
            sendHello()
        case .failed(let error):
            DispatchQueue.main.async {
                self.isConnected = false
                self.connectionStatus = "Failed: \(error.localizedDescription)"
            }
            logger.error("QUIC connection failed: \(error.localizedDescription)")
        case .cancelled:
            DispatchQueue.main.async {
                self.isConnected = false
                self.connectionStatus = "Disconnected"
            }
        case .preparing:
            DispatchQueue.main.async {
                self.connectionStatus = "Connecting..."
            }
        case .waiting(let error):
            DispatchQueue.main.async {
                self.connectionStatus = "Waiting: \(error.localizedDescription)"
            }
        default:
            break
        }
    }

    // MARK: - Handshake

    /// Send a 4-byte "HELO" to the server to fully open the bidirectional stream.
    private func sendHello() {
        guard let conn = connection else { return }

        let hello = Data("HELO".utf8)
        conn.send(content: hello, contentContext: .defaultMessage, isComplete: false,
                  completion: .contentProcessed { error in
            if let error {
                logger.error("Failed to send hello: \(error.localizedDescription)")
            } else {
                logger.info("Sent HELO to server")
            }
        })
    }

    // MARK: - Receive Pipeline

    /// Start the continuous receive loop on the connection.
    /// Called BEFORE connection.start() so receives are posted early.
    private func startReceiving(_ conn: NWConnection) {
        receiveLoop(conn)
    }

    private func receiveLoop(_ conn: NWConnection) {
        conn.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, _, isComplete, error in
            guard let self else { return }

            if let error {
                logger.debug("Receive error: \(error.localizedDescription)")
            }

            if let data, !data.isEmpty {
                logger.debug("Received \(data.count) bytes")
                self.receiveBuffer.append(data)
                self.processBuffer()
            }

            if isComplete {
                logger.info("Stream completed by server")
                return
            }

            // Continue receiving
            if self.connection != nil {
                self.receiveLoop(conn)
            }
        }
    }

    /// Process accumulated data in the receive buffer.
    /// First 12 bytes = metadata (once), then [4-byte size][payload] repeated.
    private func processBuffer() {
        // First: parse metadata if not yet received
        if metadata == nil {
            guard receiveBuffer.count >= ConnectionMetadata.size else { return }
            guard let meta = ConnectionMetadata(data: receiveBuffer) else {
                logger.error("Failed to parse metadata")
                return
            }
            logger.info("Received metadata: \(meta.width)x\(meta.height) format=\(meta.format)")
            self.metadata = meta
            DispatchQueue.main.async {
                self.connectionStatus = "Connected (\(meta.width)x\(meta.height))"
            }
            receiveBuffer = receiveBuffer.subdata(in: ConnectionMetadata.size..<receiveBuffer.count)
        }

        // Then: parse length-prefixed frames
        while receiveBuffer.count >= 4 {
            let frameSize = receiveBuffer.withUnsafeBytes {
                $0.loadUnaligned(fromByteOffset: 0, as: UInt32.self).littleEndian
            }

            let totalNeeded = 4 + Int(frameSize)
            guard receiveBuffer.count >= totalNeeded else {
                break  // Wait for more data
            }

            let framePayload = receiveBuffer.subdata(in: 4..<totalNeeded)
            receiveBuffer = receiveBuffer.subdata(in: totalNeeded..<receiveBuffer.count)

            let receiveTime = ProcessInfo.processInfo.systemUptime
            processFrame(data: framePayload, receiveTime: receiveTime)
        }
    }

    private func processFrame(data: Data, receiveTime: Double) {
        guard let header = QUICFrameHeader(data: data) else { return }

        let hevcData = data.subdata(in: QUICFrameHeader.size..<data.count)

        DispatchQueue.main.async {
            self.stats.framesReceived += 1
            self.stats.frameNumber = header.frameNumber
            self.stats.simulationTime = header.simulationTime
            self.stats.receiveTimestamp = receiveTime
        }

        // Decode HEVC on a separate queue to avoid blocking the network receive loop
        decodeQueue.async { [weak self] in
            self?.decodeHEVC(hevcData, frameNumber: header.frameNumber, receiveTime: receiveTime)
        }
    }

    // MARK: - HEVC Decode

    private func decodeHEVC(_ data: Data, frameNumber: UInt64, receiveTime: Double) {
        let decodeStart = ProcessInfo.processInfo.systemUptime

        // Parse HVCC format (4-byte length-prefixed NAL units) and extract parameter sets
        var offset = 0

        while offset + 4 < data.count {
            let length = data.withUnsafeBytes {
                CFSwapInt32BigToHost($0.loadUnaligned(fromByteOffset: offset, as: UInt32.self))
            }

            guard length > 0, offset + 4 + Int(length) <= data.count else { break }

            let nalData = data.subdata(in: offset + 4..<offset + 4 + Int(length))
            let nalType = (nalData[0] >> 1) & 0x3F  // HEVC NAL type

            switch nalType {
            case 32: vps = nalData  // VPS
            case 33: sps = nalData  // SPS
            case 34: pps = nalData  // PPS
            default: break
            }

            offset += 4 + Int(length)
        }

        // Create format description if we have parameter sets
        if let vps, let sps, let pps, formatDescription == nil {
            createFormatDescription(vps: vps, sps: sps, pps: pps)
        }

        guard let formatDesc = formatDescription else {
            logger.debug("Waiting for parameter sets (VPS/SPS/PPS)")
            return
        }

        // Create CMBlockBuffer with its own copy of the data (safe memory ownership)
        var blockBuffer: CMBlockBuffer?
        var status = CMBlockBufferCreateWithMemoryBlock(
            allocator: kCFAllocatorDefault,
            memoryBlock: nil,
            blockLength: data.count,
            blockAllocator: nil,
            customBlockSource: nil,
            offsetToData: 0,
            dataLength: data.count,
            flags: 0,
            blockBufferOut: &blockBuffer
        )

        guard status == kCMBlockBufferNoErr, let blockBuffer else {
            logger.error("Failed to create CMBlockBuffer: \(status)")
            return
        }

        // Copy HEVC data into the block buffer's own memory
        status = data.withUnsafeBytes { rawPtr in
            CMBlockBufferReplaceDataBytes(
                with: rawPtr.baseAddress!,
                blockBuffer: blockBuffer,
                offsetIntoDestination: 0,
                dataLength: data.count
            )
        }

        guard status == kCMBlockBufferNoErr else {
            logger.error("Failed to copy data into CMBlockBuffer: \(status)")
            return
        }

        var sampleBuffer: CMSampleBuffer?
        var timingInfo = CMSampleTimingInfo(
            duration: CMTime(value: 1, timescale: 30),
            presentationTimeStamp: CMTime(value: CMTimeValue(frameNumber), timescale: 30),
            decodeTimeStamp: .invalid
        )

        let sampleStatus = CMSampleBufferCreateReady(
            allocator: kCFAllocatorDefault,
            dataBuffer: blockBuffer,
            formatDescription: formatDesc,
            sampleCount: 1,
            sampleTimingEntryCount: 1,
            sampleTimingArray: &timingInfo,
            sampleSizeEntryCount: 1,
            sampleSizeArray: [data.count],
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
            DispatchQueue.main.async {
                self.stats.framesDropped += 1
            }
            return
        }

        VTDecompressionSessionWaitForAsynchronousFrames(session)

        let decodeEnd = ProcessInfo.processInfo.systemUptime
        let decodeMs = (decodeEnd - decodeStart) * 1000

        DispatchQueue.main.async {
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

    private func handleDecodedFrame(_ pixelBuffer: CVPixelBuffer) {
        var cgImage: CGImage?
        VTCreateCGImageFromCVPixelBuffer(pixelBuffer, options: nil, imageOut: &cgImage)

        if let cgImage {
            DispatchQueue.main.async {
                self.currentFrame = cgImage
            }
        }
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
