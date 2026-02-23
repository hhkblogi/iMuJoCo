// HevcReceiverApp.swift
// macOS SwiftUI app for receiving HEVC-over-RTP video from iMuJoCo.
//
// Usage:
//   HevcReceiver --host <ip> --port <port>
//   Defaults: localhost:9100

import SwiftUI

@main
struct HevcReceiverApp: App {
    var body: some Scene {
        WindowGroup {
            HevcReceiverView(host: Self.host, port: Self.port)
                .frame(minWidth: 480, minHeight: 360)
        }
        .defaultSize(width: 640, height: 520)
    }

    // MARK: - CLI Arguments

    private static var host: String {
        let args = CommandLine.arguments
        if let idx = args.firstIndex(of: "--host"), idx + 1 < args.count {
            return args[idx + 1]
        }
        return "localhost"
    }

    private static var port: UInt16 {
        let args = CommandLine.arguments
        if let idx = args.firstIndex(of: "--port"), idx + 1 < args.count {
            return UInt16(args[idx + 1]) ?? 9100
        }
        return 9100
    }
}
