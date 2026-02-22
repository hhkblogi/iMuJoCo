// mjc_hevc_encoder.swift
// H.265/HEVC encoder using VideoToolbox hardware encoding.
// Zero-copy pipeline: wraps UMA MTLBuffer pointer as CVPixelBuffer (no pixel alloc),
// outputs HVCC format (4-byte length-prefixed NAL units) into a pre-allocated buffer.

import CoreVideo
import Foundation
import VideoToolbox
import os.log

private let logger = Logger(subsystem: "com.mujoco.video", category: "HEVCEncoder")

// MARK: - VTCompressionSession output callback (C function pointer)

private func hevcOutputCallback(
    outputCallbackRefCon: UnsafeMutableRawPointer?,
    sourceFrameRefCon: UnsafeMutableRawPointer?,
    status: OSStatus,
    infoFlags: UInt32,
    sampleBuffer: CMSampleBuffer?
) {
    guard let refCon = outputCallbackRefCon else { return }
    let encoder = Unmanaged<MJCHEVCEncoder>.fromOpaque(refCon).takeUnretainedValue()
    encoder.handleEncodedFrame(status: status, flags: VTEncodeInfoFlags(rawValue: infoFlags), sampleBuffer: sampleBuffer)
}

// MARK: - HEVC Encoder

/// Hardware H.265/HEVC encoder using VideoToolbox.
///
/// Zero-copy input: wraps the caller's BGRA MTLBuffer pointer directly as a
/// CVPixelBuffer (metadata only, no pixel allocation or channel swizzle).
/// Output: HVCC format (4-byte big-endian length-prefixed NAL units) into a
/// pre-allocated buffer — no per-frame allocation.
/// Synchronous: one frame in, one frame out (no pipeline buffering).
public final class MJCHEVCEncoder: MJCVideoEncoder {
    public var format: UInt8 { 4 }  // MJVideoFormat::HEVC

    private var session: VTCompressionSession?
    private let width: Int
    private let height: Int

    /// HVCC output from the most recent encode call (set by output callback).
    /// Non-owning view into `outputBuffer` — valid until next `encode()` call.
    fileprivate var encodedOutput: Data?

    /// Pre-allocated output buffer for compressed bitstream (reused across frames).
    private let outputBuffer = NSMutableData(capacity: 64 * 1024) ?? NSMutableData()

    /// VPS/SPS/PPS parameter sets in HVCC format (4-byte length prefix).
    private var parameterSetData: Data?

    /// Public access to raw parameter sets for SDP generation.
    public private(set) var parameterSets: [Data] = []

    /// Frame counter for timestamp generation.
    private var frameCount: Int64 = 0

    public init(width: Int, height: Int, bitrate: Int = 2_000_000) {
        self.width = width
        self.height = height

        // Create session using C-style callback
        var session: VTCompressionSession?
        let selfPtr = Unmanaged.passUnretained(self).toOpaque()

        let status = VTCompressionSessionCreate(
            allocator: kCFAllocatorDefault,
            width: Int32(width),
            height: Int32(height),
            codecType: kCMVideoCodecType_HEVC,
            encoderSpecification: [
                kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder: true
            ] as CFDictionary,
            imageBufferAttributes: [
                kCVPixelBufferPixelFormatTypeKey: kCVPixelFormatType_32BGRA,
                kCVPixelBufferWidthKey: width,
                kCVPixelBufferHeightKey: height,
            ] as CFDictionary,
            compressedDataAllocator: nil,
            outputCallback: hevcOutputCallback,
            refcon: selfPtr,
            compressionSessionOut: &session
        )

        guard status == noErr, let session else {
            logger.error("Failed to create VTCompressionSession: \(status)")
            return
        }

        self.session = session

        // Configure session properties
        VTSessionSetProperty(session, key: kVTCompressionPropertyKey_RealTime, value: kCFBooleanTrue)
        VTSessionSetProperty(session, key: kVTCompressionPropertyKey_ProfileLevel,
                             value: kVTProfileLevel_HEVC_Main_AutoLevel)
        VTSessionSetProperty(session, key: kVTCompressionPropertyKey_AllowFrameReordering,
                             value: kCFBooleanFalse)
        VTSessionSetProperty(session, key: kVTCompressionPropertyKey_AverageBitRate,
                             value: bitrate as CFNumber)
        VTSessionSetProperty(session, key: kVTCompressionPropertyKey_MaxKeyFrameInterval,
                             value: 60 as CFNumber)

        VTCompressionSessionPrepareToEncodeFrames(session)
        logger.info("HEVC encoder created: \(width)x\(height) @ \(bitrate / 1000) kbps")
    }

    deinit {
        if let session {
            VTCompressionSessionInvalidate(session)
        }
    }

    public func encode(data: Data, width: Int, height: Int) -> Data? {
        guard let session else { return nil }
        guard width == self.width && height == self.height else {
            logger.error("HEVC encode size mismatch: got \(width)x\(height), expected \(self.width)x\(self.height)")
            return nil
        }

        let bytesPerRow = width * 4
        let expectedSize = bytesPerRow * height
        guard data.count >= expectedSize else { return nil }

        // Zero-copy: wrap the caller's BGRA pixel pointer as a CVPixelBuffer.
        // No pixel allocation, no channel swizzle — just a metadata object.
        // Safe because VTCompressionSessionCompleteFrames is synchronous below.
        var pixelBuffer: CVPixelBuffer?
        let cvStatus = data.withUnsafeBytes { rawPtr -> CVReturn in
            CVPixelBufferCreateWithBytes(
                kCFAllocatorDefault,
                width, height,
                kCVPixelFormatType_32BGRA,
                UnsafeMutableRawPointer(mutating: rawPtr.baseAddress!),
                bytesPerRow,
                nil, nil, nil,
                &pixelBuffer
            )
        }
        guard cvStatus == kCVReturnSuccess, let pixelBuffer else { return nil }

        // Create presentation timestamp
        let pts = CMTime(value: frameCount, timescale: 90000)
        frameCount += 3000  // 30fps at 90kHz clock

        // Encode synchronously
        encodedOutput = nil

        let encodeStatus = VTCompressionSessionEncodeFrame(
            session,
            imageBuffer: pixelBuffer,
            presentationTimeStamp: pts,
            duration: CMTime(value: 3000, timescale: 90000),
            frameProperties: nil,
            sourceFrameRefcon: nil,
            infoFlagsOut: nil
        )

        guard encodeStatus == noErr else {
            logger.error("VTCompressionSessionEncodeFrame failed: \(encodeStatus)")
            return nil
        }

        // Force synchronous completion — encoding finishes before pixel pointer scope ends
        let completeStatus = VTCompressionSessionCompleteFrames(session, untilPresentationTimeStamp: pts)
        if completeStatus != noErr {
            logger.error("VTCompressionSessionCompleteFrames failed: \(completeStatus)")
            return nil
        }

        return encodedOutput
    }

    // MARK: - Private

    /// Output callback invoked by VideoToolbox when a frame is encoded.
    /// Writes HVCC format (4-byte length-prefixed NALUs) into `outputBuffer`.
    fileprivate func handleEncodedFrame(status: OSStatus, flags: VTEncodeInfoFlags, sampleBuffer: CMSampleBuffer?) {
        guard status == noErr, let sampleBuffer else {
            if status != noErr {
                logger.error("Encode output error: \(status)")
            }
            return
        }

        let isKeyFrame = sampleBuffer.sampleAttachments.first?[.notSync] == nil

        // Reset pre-allocated buffer for this frame
        outputBuffer.length = 0

        // On IDR frames, prepend VPS/SPS/PPS as length-prefixed NALUs
        if isKeyFrame {
            extractParameterSets(from: sampleBuffer)
            if let psData = parameterSetData {
                outputBuffer.append(psData)
            }
        }

        // Append CMBlockBuffer data directly — already in HVCC format
        // (4-byte big-endian length prefix + NAL unit data)
        guard let dataBuffer = sampleBuffer.dataBuffer else { return }

        var totalLength: Int = 0
        var dataPointer: UnsafeMutablePointer<CChar>?
        let blockStatus = CMBlockBufferGetDataPointer(
            dataBuffer, atOffset: 0, lengthAtOffsetOut: nil,
            totalLengthOut: &totalLength, dataPointerOut: &dataPointer
        )
        guard blockStatus == kCMBlockBufferNoErr, let dataPointer else { return }

        outputBuffer.append(dataPointer, length: totalLength)

        // Non-owning view into outputBuffer — valid until next encode() call
        encodedOutput = Data(bytesNoCopy: outputBuffer.mutableBytes,
                             count: outputBuffer.length,
                             deallocator: .none)
    }

    /// Extract VPS/SPS/PPS from CMFormatDescription and store as HVCC format
    /// (4-byte big-endian length prefix + NAL data).
    private func extractParameterSets(from sampleBuffer: CMSampleBuffer) {
        // Already extracted — skip
        if parameterSetData != nil { return }

        guard let formatDesc = sampleBuffer.formatDescription else { return }

        var output = Data()
        var sets: [Data] = []

        // HEVC has 3 parameter set types: VPS(0), SPS(1), PPS(2)
        for index in 0..<3 {
            var parameterSetPtr: UnsafePointer<UInt8>?
            var parameterSetSize: Int = 0
            let status = CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
                formatDesc,
                parameterSetIndex: index,
                parameterSetPointerOut: &parameterSetPtr,
                parameterSetSizeOut: &parameterSetSize,
                parameterSetCountOut: nil,
                nalUnitHeaderLengthOut: nil
            )
            guard status == noErr, let ptr = parameterSetPtr else { continue }

            let setData = Data(bytes: ptr, count: parameterSetSize)
            sets.append(setData)

            // 4-byte big-endian length prefix (HVCC format)
            var length = CFSwapInt32HostToBig(UInt32(parameterSetSize))
            output.append(Data(bytes: &length, count: 4))
            output.append(setData)
        }

        parameterSets = sets
        parameterSetData = output
        logger.info("Extracted \(sets.count) HEVC parameter sets (VPS/SPS/PPS)")
    }
}
