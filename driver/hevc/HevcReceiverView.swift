// HevcReceiverView.swift
// Main view: video feed with latency stats overlay.

import SwiftUI

struct HevcReceiverView: View {
    @State private var client = HevcVideoClient()
    @State private var host: String
    @State private var port: String

    init(host: String = "localhost", port: UInt16 = 9100) {
        _host = State(initialValue: host)
        _port = State(initialValue: String(port))
    }

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()

            // Video frame
            if let frame = client.currentFrame {
                Image(decorative: frame, scale: 1.0)
                    .resizable()
                    .aspectRatio(contentMode: .fit)
            } else {
                VStack(spacing: 16) {
                    Image(systemName: "video.slash")
                        .font(.system(size: 48))
                        .foregroundColor(.gray)
                    Text("No video signal")
                        .foregroundColor(.gray)
                }
            }

            // Stats overlay (top-right)
            VStack {
                HStack {
                    Spacer()
                    statsOverlay
                }
                Spacer()
            }
            .padding()

            // Connection controls (bottom)
            VStack {
                Spacer()
                connectionBar
            }
            .padding()
        }
    }

    // MARK: - Stats Overlay

    private var statsOverlay: some View {
        VStack(alignment: .trailing, spacing: 4) {
            if client.isConnected {
                Text("Frame #\(client.stats.frameNumber)")
                    .font(.system(size: 11, design: .monospaced))
                Text(String(format: "Sim Time: %.3fs", client.stats.simulationTime))
                    .font(.system(size: 11, design: .monospaced))
                Text(String(format: "Decode: %.1fms", client.stats.decodeLatencyMs))
                    .font(.system(size: 11, design: .monospaced))
                    .foregroundColor(client.stats.decodeLatencyMs < 10 ? .green : .yellow)
                Text("Received: \(client.stats.framesReceived)")
                    .font(.system(size: 11, design: .monospaced))
                Text("Decoded: \(client.stats.framesDecoded)")
                    .font(.system(size: 11, design: .monospaced))
                if client.stats.framesDropped > 0 {
                    Text("Dropped: \(client.stats.framesDropped)")
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundColor(.red)
                }
            }
        }
        .foregroundColor(.white.opacity(0.9))
        .padding(8)
        .background(Color.black.opacity(0.6))
        .clipShape(RoundedRectangle(cornerRadius: 8))
    }

    // MARK: - Connection Bar

    private var connectionBar: some View {
        HStack(spacing: 12) {
            // Status indicator
            Circle()
                .fill(client.isConnected ? Color.green : client.isConnecting ? Color.yellow : Color.red)
                .frame(width: 10, height: 10)

            Text(client.connectionStatus)
                .font(.system(size: 12, design: .monospaced))
                .foregroundColor(.white.opacity(0.8))
                .lineLimit(1)

            Spacer()

            // Host:Port fields
            TextField("Host", text: $host)
                .font(.system(size: 12, design: .monospaced))
                .textFieldStyle(.roundedBorder)
                .frame(width: 140)

            TextField("Port", text: $port)
                .font(.system(size: 12, design: .monospaced))
                .textFieldStyle(.roundedBorder)
                .frame(width: 60)

            // Connect/Disconnect/Cancel button
            Button(action: {
                if client.isConnected || client.isConnecting {
                    client.disconnect()
                } else {
                    let portNum = UInt16(port) ?? 9100
                    client.connect(host: host, port: portNum)
                }
            }) {
                Text(client.isConnected ? "Disconnect" : client.isConnecting ? "Cancel" : "Connect")
                    .font(.system(size: 12, weight: .semibold))
            }
            .buttonStyle(.borderedProminent)
            .tint(client.isConnected ? .red : client.isConnecting ? .orange : .blue)
        }
        .padding(10)
        .background(Color.black.opacity(0.7))
        .clipShape(RoundedRectangle(cornerRadius: 10))
    }
}
