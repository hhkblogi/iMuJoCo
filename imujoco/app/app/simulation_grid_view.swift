// simulation_grid_view.swift
// 2x2 grid view for multiple simulations

import Darwin
import SwiftUI

// MARK: - Device IP Helper

func getDeviceIPAddress() -> String? {
    var address: String?
    var ifaddr: UnsafeMutablePointer<ifaddrs>?

    guard getifaddrs(&ifaddr) == 0, let firstAddr = ifaddr else {
        return nil
    }

    defer { freeifaddrs(ifaddr) }

    for ptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
        let interface = ptr.pointee

        // Skip interfaces with nil address
        guard let ifaAddr = interface.ifa_addr else { continue }
        let addrFamily = ifaAddr.pointee.sa_family

        // Check for IPv4 (AF_INET)
        if addrFamily == UInt8(AF_INET) {
            let name = String(cString: interface.ifa_name)

            // Look for WiFi (en0) or cellular (pdp_ip0)
            if name == "en0" || name.hasPrefix("pdp_ip") {
                var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                if getnameinfo(
                    ifaAddr,
                    socklen_t(ifaAddr.pointee.sa_len),
                    &hostname,
                    socklen_t(hostname.count),
                    nil,
                    0,
                    NI_NUMERICHOST
                ) == 0 {
                    // Find null terminator and create string
                    if let nullIndex = hostname.firstIndex(of: 0) {
                        address = String(decoding: hostname[..<nullIndex].map { UInt8(bitPattern: $0) }, as: UTF8.self)
                    }
                    if name == "en0" { break }  // Prefer WiFi
                }
            }
        }
    }

    return address
}

// MARK: - Grid View

struct SimulationGridView: View {
    @Bindable var gridManager: SimulationGridManager
    @State private var showingModelPicker = false
    @State private var selectedCellIndex: Int = 0
    @State private var deviceIP: String = "..."
    @State private var showingErrorAlert = false
    @State private var errorMessage: String = ""

    let columns = [
        GridItem(.flexible(), spacing: 8),
        GridItem(.flexible(), spacing: 8)
    ]

    var body: some View {
        VStack(spacing: 0) {
            // Menu bar with device IP
            menuBar

            // Grid of simulations
            GeometryReader { geometry in
                LazyVGrid(columns: columns, spacing: 8) {
                    ForEach(0..<4) { index in
                        if let instance = gridManager.instance(at: index) {
                            SimulationCellView(
                                instance: instance,
                                onTapFullscreen: {
                                    gridManager.enterFullscreen(index: index)
                                },
                                onLoadModel: {
                                    selectedCellIndex = index
                                    showingModelPicker = true
                                }
                            )
                            .frame(height: (geometry.size.height - 8) / 2)
                        }
                    }
                }
                .padding(8)
            }
        }
        .background(Color.black)
        .onAppear {
            updateDeviceIP()
        }
        #if os(iOS)
        .sheet(isPresented: $showingModelPicker) {
            ModelPickerView(
                modelNames: gridManager.bundledModelNames,
                onSelectModel: { modelName in
                    loadModel(name: modelName)
                },
                onDismiss: {
                    showingModelPicker = false
                }
            )
        }
        #endif
        #if os(tvOS)
        .fullScreenCover(isPresented: $showingModelPicker) {
            TVModelPickerView(
                modelNames: gridManager.bundledModelNames,
                onSelectModel: { modelName in
                    loadModel(name: modelName)
                },
                onDismiss: {
                    showingModelPicker = false
                }
            )
        }
        #endif
        #if os(macOS)
        .sheet(isPresented: $showingModelPicker) {
            ModelPickerView(
                modelNames: gridManager.bundledModelNames,
                onSelectModel: { modelName in
                    loadModel(name: modelName)
                },
                onDismiss: {
                    showingModelPicker = false
                }
            )
        }
        #endif
        .alert("Failed to Load Model", isPresented: $showingErrorAlert) {
            Button("OK", role: .cancel) {}
        } message: {
            Text(errorMessage)
        }
    }

    // MARK: - Model Loading

    private func loadModel(name: String) {
        showingModelPicker = false
        Task {
            do {
                try await gridManager.loadBundledModel(at: selectedCellIndex, name: name)
            } catch {
                await MainActor.run {
                    errorMessage = error.localizedDescription
                    showingErrorAlert = true
                }
            }
        }
    }

    // MARK: - Menu Bar

    private var menuBar: some View {
        HStack {
            // App title
            Text("iMuJoCo")
                .font(.headline)
                .fontWeight(.bold)
                .foregroundColor(.white)

            Spacer()

            // Device IP and ports
            HStack(spacing: 12) {
                // Network icon
                Image(systemName: "network")
                    .foregroundColor(.green)

                // IP Address
                Text(deviceIP)
                    .font(.system(.subheadline, design: .monospaced))
                    .foregroundColor(.white)
            }
            .padding(.horizontal, 12)
            .padding(.vertical, 6)
            .background(Color.white.opacity(0.1))
            .clipShape(Capsule())
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 10)
        .background(Color.black.opacity(0.8))
    }

    private func updateDeviceIP() {
        if let ip = getDeviceIPAddress() {
            deviceIP = ip
        } else {
            deviceIP = "No Network"
        }
    }
}

// MARK: - Model Picker

struct ModelPickerView: View {
    let modelNames: [String]
    var onSelectModel: (String) -> Void
    var onDismiss: () -> Void

    var body: some View {
        NavigationStack {
            List(modelNames, id: \.self) { name in
                Button(action: { onSelectModel(name) }) {
                    HStack {
                        Image(systemName: "cube.fill")
                            .foregroundColor(.blue)
                        Text(name.capitalized)
                        Spacer()
                    }
                    .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
            }
            .navigationTitle("Select Model")
            #if os(iOS)
            .navigationBarTitleDisplayMode(.inline)
            #endif
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel", action: onDismiss)
                }
            }
        }
        #if os(iOS)
        .presentationDetents([.medium])
        #endif
        #if os(macOS)
        .frame(minWidth: 300, minHeight: 250)
        #endif
    }
}

// MARK: - tvOS Model Picker

#if os(tvOS)
struct TVModelPickerView: View {
    let modelNames: [String]
    var onSelectModel: (String) -> Void
    var onDismiss: () -> Void

    var body: some View {
        ZStack {
            // Dimmed background
            Color.black.opacity(0.7)
                .ignoresSafeArea()

            // Alert-style dialog
            VStack(spacing: 0) {
                // Title
                Text("Select Model")
                    .font(.headline)
                    .padding(.vertical, 20)

                Divider()

                // Model buttons
                ForEach(modelNames, id: \.self) { name in
                    Button(action: { onSelectModel(name) }) {
                        Text(name.capitalized)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 16)
                    }

                    Divider()
                }

                // Cancel button
                Button(action: onDismiss) {
                    Text("Cancel")
                        .fontWeight(.semibold)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                }
            }
            .frame(width: 400)
            .background(Color(white: 0.2))
            .clipShape(RoundedRectangle(cornerRadius: 16))
        }
        .onExitCommand {
            onDismiss()
        }
    }
}
#endif

// MARK: - Preview

#if DEBUG
#Preview {
    SimulationGridView(gridManager: SimulationGridManager())
}
#endif
