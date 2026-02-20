// mjc_jpeg_encoder.swift
// JPEG encoder using CoreGraphics for hardware-accelerated compression.
// Converts raw RGBA8 pixel data to JPEG format.

import Foundation
import CoreGraphics
import ImageIO
import UniformTypeIdentifiers

// MARK: - JPEG Encoder

/// JPEG encoder using CoreGraphics.
/// Compresses raw RGBA8 pixel data into JPEG format.
/// At 256x256 this typically takes < 1ms on Apple Silicon.
public final class MJCJPEGEncoder: MJCVideoEncoder {
    public var format: UInt8 { 3 }  // MJVideoFormat::JPEG

    /// JPEG compression quality (0.0 = max compression, 1.0 = max quality)
    private let quality: CGFloat

    /// Reusable color space
    private let colorSpace = CGColorSpaceCreateDeviceRGB()

    public init(quality: CGFloat = 0.8) {
        self.quality = max(0.0, min(1.0, quality))
    }

    public func encode(data: Data, width: Int, height: Int) -> Data? {
        let bytesPerRow = width * 4
        let expectedSize = bytesPerRow * height
        guard data.count >= expectedSize else { return nil }

        // Create CGImage from raw RGBA8 data
        guard let provider = CGDataProvider(data: data as CFData) else { return nil }
        guard let image = CGImage(
            width: width,
            height: height,
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: bytesPerRow,
            space: colorSpace,
            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
            provider: provider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        ) else { return nil }

        // Encode to JPEG
        let jpegData = NSMutableData()
        guard let destination = CGImageDestinationCreateWithData(
            jpegData as CFMutableData,
            UTType.jpeg.identifier as CFString,
            1,
            nil
        ) else { return nil }

        let options: [CFString: Any] = [
            kCGImageDestinationLossyCompressionQuality: quality
        ]
        CGImageDestinationAddImage(destination, image, options as CFDictionary)

        guard CGImageDestinationFinalize(destination) else { return nil }
        return jpegData as Data
    }
}
