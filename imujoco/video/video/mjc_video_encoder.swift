// mjc_video_encoder.swift
// Video encoder protocol and raw (passthrough) encoder.
// JPEG encoder added in PR 3.

import Foundation

// MARK: - Encoder Protocol

/// Protocol for encoding rendered pixel data before transport.
///
/// Encoders transform raw RGBA pixel data into a format suitable for transmission.
/// The raw encoder passes through unchanged; JPEG encoder compresses (PR 3).
public protocol MJCVideoEncoder {
    /// The video format identifier for the encoded output.
    var format: UInt8 { get }

    /// Encode raw RGBA pixel data.
    /// - Parameters:
    ///   - data: Raw RGBA8 pixel data (width * height * 4 bytes)
    ///   - width: Image width in pixels
    ///   - height: Image height in pixels
    /// - Returns: Encoded data, or nil on failure
    func encode(data: Data, width: Int, height: Int) -> Data?
}

// MARK: - Raw Encoder

/// Passthrough encoder — sends raw RGBA8 pixel data unchanged.
/// Zero overhead, maximum bandwidth. Use when network bandwidth isn't constrained.
public final class MJCRawEncoder: MJCVideoEncoder {
    public var format: UInt8 { 0 }  // MJVideoFormat::RGBA8

    public init() {}

    public func encode(data: Data, width: Int, height: Int) -> Data? {
        return data
    }
}
