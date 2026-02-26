// simulation_grid_view.swift
// 2x2 grid view for multiple simulations

import Darwin
import SwiftUI

// MARK: - Device IP Helper

/// Returns all IPv4 addresses on en* (WiFi/Ethernet) interfaces.
func getAllLocalAddresses() -> [String] {
    var addresses: [String] = []
    var ifaddr: UnsafeMutablePointer<ifaddrs>?

    guard getifaddrs(&ifaddr) == 0, let firstAddr = ifaddr else {
        return addresses
    }

    defer { freeifaddrs(ifaddr) }

    for ptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
        let interface = ptr.pointee

        guard let ifaAddr = interface.ifa_addr else { continue }
        let addrFamily = ifaAddr.pointee.sa_family

        if addrFamily == UInt8(AF_INET) {
            let name = String(cString: interface.ifa_name)
            guard name.hasPrefix("en") else { continue }

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
                if let nullIndex = hostname.firstIndex(of: 0) {
                    let ip = String(decoding: hostname[..<nullIndex].map { UInt8(bitPattern: $0) }, as: UTF8.self)
                    addresses.append(ip)
                }
            }
        }
    }

    return addresses
}

// MARK: - Grid View

struct SimulationGridView: View {
    @Bindable var gridManager: SimulationGridManager
    @State private var showingModelPicker = false
    @State private var selectedCellIndex: Int = 0
    @State private var deviceIP: String = "..."
    @State private var availableIPs: [String] = []
    @State private var showingErrorAlert = false
    @State private var errorMessage: String = ""
    @State private var showingSettings = false
    @AppStorage("defaultView") private var defaultView: Int = 0
    @AppStorage("bindAddress") private var bindAddress: String = ""
    #if os(iOS)
    @AppStorage("caffeineMode") private var caffeineMode: Int = 1
    #endif
    @AppStorage("defaultLocked") private var defaultLocked: Bool = true
    @AppStorage("showStatsBar") private var showStatsBar: Bool = true
    @AppStorage("videoTransport") private var videoTransport: Int = 0
    #if os(macOS)
    @Environment(\.openSettings) private var openSettings
    #endif

    let columns = [
        GridItem(.flexible(), spacing: 8),
        GridItem(.flexible(), spacing: 8)
    ]

    var body: some View {
        VStack(spacing: 0) {
            // Menu bar with device IP
            menuBar

            if showStatsBar {
                PerformanceStatsBar(instances: gridManager.instances, grpcRpcCount: { gridManager.grpcRpcCount })
            }

            // Grid of simulations
            GeometryReader { geometry in
                LazyVGrid(columns: columns, spacing: 8) {
                    ForEach(0..<4) { index in
                        if let instance = gridManager.instance(at: index) {
                            SimulationCellView(
                                instance: instance,
                                instanceIndex: index,
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
        // videoTransport is persisted via @AppStorage; new instances
        // read it in start(). Changing it does NOT restart running streamers
        // — use the per-instance MJPEG/HEVC toggle for that.
        #if !os(tvOS)
        .sheet(isPresented: $showingModelPicker) {
            ModelPickerView(
                modelGroups: gridManager.bundledModelsBySource,
                onSelectModel: { modelName in
                    loadModel(name: modelName)
                },
                onDismiss: {
                    showingModelPicker = false
                }
            )
        }
        #else
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
        .alert("Failed to Load Model", isPresented: $showingErrorAlert) {
            Button("OK", role: .cancel) {}
        } message: {
            Text(errorMessage)
        }
        #if !os(macOS)
        .sheet(isPresented: $showingSettings) {
            SettingsView(defaultView: $defaultView, defaultLocked: $defaultLocked, onDismiss: { showingSettings = false }, gridManager: gridManager)
        }
        #endif
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

    private var ipIconColor: Color {
        deviceIP == "No Network" ? .red : .green
    }

    private var menuBar: some View {
        HStack {
            // Settings button
            Button(action: {
                #if os(macOS)
                openSettings()
                #else
                showingSettings = true
                #endif
            }) {
                Image(systemName: "slider.horizontal.3")
                    .font(.system(size: 18, weight: .medium))
                    .foregroundColor(.white)
            }
            .buttonStyle(.plain)

            Spacer()

            // Caffeine mode indicator
            #if os(iOS)
            if caffeineMode >= 1 {
                Image(systemName: caffeineMode >= 2 ? "cup.and.heat.waves.fill" : "cup.and.saucer.fill")
                    .font(.system(size: 14))
                    .foregroundColor(.white)
                    .padding(.trailing, 6)
            }
            #endif

            // IP address picker capsule (tap opens dropdown)
            Menu {
                ForEach(availableIPs, id: \.self) { ip in
                    Button {
                        bindAddress = ip
                        deviceIP = ip
                    } label: {
                        if ip == deviceIP {
                            Label(ip, systemImage: "checkmark")
                        } else {
                            Text(ip)
                        }
                    }
                }
                if !availableIPs.isEmpty { Divider() }
                Button {
                    refreshIPs()
                } label: {
                    Label("Refresh", systemImage: "arrow.clockwise")
                }
            } label: {
                HStack(spacing: 8) {
                    Image(systemName: "network")
                        .foregroundColor(ipIconColor)
                    Text(deviceIP)
                        .font(.system(size: 11, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                }
                .padding(.horizontal, 12)
                .padding(.vertical, 6)
                .background(Color.white.opacity(0.1))
                .clipShape(Capsule())
            }
        }
        .padding(.horizontal, 16)
        .padding(.top, 10)
        .padding(.bottom, 4)
        .background(Color.black.opacity(0.8))
    }

    private func updateDeviceIP() {
        Task.detached {
            let ips = getAllLocalAddresses()
            await MainActor.run {
                availableIPs = ips
                if !bindAddress.isEmpty, ips.contains(bindAddress) {
                    deviceIP = bindAddress
                } else if let first = ips.first {
                    deviceIP = first
                    bindAddress = first
                } else {
                    deviceIP = "No Network"
                }
            }
        }
    }

    private func refreshIPs() {
        updateDeviceIP()
    }

}

// MARK: - Model Picker

private struct PressableRow<Content: View>: View {
    let action: () -> Void
    @ViewBuilder let content: () -> Content

    var body: some View {
        Button(action: action) {
            content()
        }
        .buttonStyle(PressableButtonStyle())
    }
}

private struct PressableButtonStyle: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .contentShape(Rectangle())
            .opacity(configuration.isPressed ? 0.5 : 1.0)
    }
}

struct ModelPickerView: View {
    let modelGroups: [(source: ModelSource, models: [BundledModel])]
    // Legacy support: flat name list (converted to ungrouped)
    let modelNames: [String]
    var onSelectModel: (String) -> Void
    var onDismiss: () -> Void

    init(modelGroups: [(source: ModelSource, models: [BundledModel])],
         onSelectModel: @escaping (String) -> Void,
         onDismiss: @escaping () -> Void) {
        self.modelGroups = modelGroups
        self.modelNames = []
        self.onSelectModel = onSelectModel
        self.onDismiss = onDismiss
    }

    init(modelNames: [String],
         onSelectModel: @escaping (String) -> Void,
         onDismiss: @escaping () -> Void) {
        self.modelGroups = []
        self.modelNames = modelNames
        self.onSelectModel = onSelectModel
        self.onDismiss = onDismiss
    }

    var body: some View {
        NavigationStack {
            List {
                if !modelGroups.isEmpty {
                    ForEach(modelGroups, id: \.source) { group in
                        Section {
                            ForEach(group.models, id: \.name) { model in
                                PressableRow(action: { onSelectModel(model.name) }) {
                                    HStack {
                                        Text(model.name)
                                        Spacer()
                                    }
                                    .padding(.vertical, 2)
                                }
                            }
                        } header: {
                            Text(group.source.rawValue)
                        }
                    }
                } else {
                    ForEach(modelNames, id: \.self) { name in
                        PressableRow(action: { onSelectModel(name) }) {
                            HStack {
                                Image(systemName: "cube.fill")
                                    .foregroundColor(.blue)
                                Text(name)
                                Spacer()
                            }
                        }
                    }
                }
            }
            .navigationTitle("Select Model")
            #if os(iOS)
            .listStyle(.insetGrouped)
            #elseif os(tvOS)
            .listStyle(.grouped)
            #endif
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
        .presentationDetents([.medium, .large])
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
                        Text(name)
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

// MARK: - Settings View

/// Mini layout diagram showing a 2x2 grid with optional cell highlight.
/// `highlightedCell` nil = all cells equal (grid), 0-3 = specific cell for fullscreen.
struct LayoutIcon: View {
    let highlightedCell: Int?  // nil = grid view, 0-3 = fullscreen instance
    let isSelected: Bool
    var size: CGFloat = 36
    var tintColor: Color? = nil  // nil = default blue/gray

    private let gap: CGFloat = 2
    private let cornerR: CGFloat = 2

    private var activeColor: Color {
        if let tint = tintColor { return tint }
        return isSelected ? Color.blue : Color.gray
    }

    private var bgColor: Color {
        if tintColor != nil { return isSelected ? activeColor.opacity(0.15) : Color.clear }
        return isSelected ? Color.blue.opacity(0.2) : Color.clear
    }

    private var borderColor: Color {
        if tintColor != nil { return isSelected ? activeColor.opacity(0.4) : activeColor.opacity(0.2) }
        return isSelected ? Color.blue : Color.gray.opacity(0.3)
    }

    var body: some View {
        let cellSize = (size - gap) / 2

        VStack(spacing: gap) {
            HStack(spacing: gap) {
                cell(0, cellSize: cellSize)
                cell(1, cellSize: cellSize)
            }
            HStack(spacing: gap) {
                cell(2, cellSize: cellSize)
                cell(3, cellSize: cellSize)
            }
        }
        .padding(size * 0.16)
        .background(
            RoundedRectangle(cornerRadius: size * 0.16)
                .fill(bgColor)
        )
        .overlay(
            RoundedRectangle(cornerRadius: size * 0.16)
                .stroke(borderColor, lineWidth: isSelected ? 1.5 : 1)
        )
    }

    @ViewBuilder
    private func cell(_ index: Int, cellSize: CGFloat) -> some View {
        let active = highlightedCell == nil || highlightedCell == index
        RoundedRectangle(cornerRadius: cornerR)
            .fill(active ? activeColor : activeColor.opacity(0.2))
            .frame(width: cellSize, height: cellSize)
    }
}

struct SettingsView: View {
    @Binding var defaultView: Int
    @Binding var defaultLocked: Bool
    var onDismiss: () -> Void
    var gridManager: SimulationGridManager?
    @AppStorage("caffeineMode") private var caffeineMode: Int = 1  // 0=off, 1=half, 2=full
    @AppStorage("tripleClickAction") private var tripleClickAction: Int = 0  // 0=grid/fullscreen, 1=lock/unlock
    @AppStorage("showStatsBar") private var showStatsBar: Bool = true
    @AppStorage("videoTransport") private var videoTransport: Int = 0  // 0=MJPEG/HTTP, 1=RTP/RTSP, 2=HEVC/QUIC
    @AppStorage("bindAddress") private var bindAddress: String = ""
    @AppStorage("grpcPort") private var grpcPort: Int = 8999
    @AppStorage("inst1_udpPort") private var inst1UdpPort: Int = 9001
    @AppStorage("inst2_udpPort") private var inst2UdpPort: Int = 9002
    @AppStorage("inst3_udpPort") private var inst3UdpPort: Int = 9003
    @AppStorage("inst4_udpPort") private var inst4UdpPort: Int = 9004
    @AppStorage("inst1_camPort") private var inst1CamPort: Int = 9100
    @AppStorage("inst2_camPort") private var inst2CamPort: Int = 9200
    @AppStorage("inst3_camPort") private var inst3CamPort: Int = 9300
    @AppStorage("inst4_camPort") private var inst4CamPort: Int = 9400
    @State private var showCaffeineInfo = false
    @State private var showVideoTransportInfo = false
    @State private var showPortWarning = false
    @State private var portWarningMessage = ""
    @State private var editingPort: PortEditTarget?
    @State private var showGrpcPortInfo = false
    @State private var showUdpPortInfo = false
    @State private var showCamPortInfo = false

    // tag 0 = grid, 1-4 = fullscreen instance (highlightedCell 0-3)
    private let viewOptions: [(highlightedCell: Int?, tag: Int)] = [
        (nil, 0),
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4),
    ]

    private func tripleClickOption<Content: View>(tag: Int, @ViewBuilder content: () -> Content) -> some View {
        Button(action: { tripleClickAction = tag }) {
            content()
                .frame(maxWidth: .infinity)
                .padding(.vertical, 8)
                .background(
                    RoundedRectangle(cornerRadius: 8)
                        .fill(tripleClickAction == tag ? Color.blue.opacity(0.2) : Color.clear)
                )
                .overlay(
                    RoundedRectangle(cornerRadius: 8)
                        .stroke(tripleClickAction == tag ? Color.blue : Color.gray.opacity(0.3), lineWidth: tripleClickAction == tag ? 1.5 : 1)
                )
                .foregroundColor(tripleClickAction == tag ? .blue : .gray)
        }
        .buttonStyle(.plain)
    }

    var body: some View {
        NavigationStack {
            List {
                Section {
                    VStack(alignment: .leading, spacing: 10) {
                        Text("Default View")
                            .font(.subheadline)
                        HStack {
                            ForEach(viewOptions, id: \.tag) { option in
                                Button(action: { defaultView = option.tag }) {
                                    LayoutIcon(
                                        highlightedCell: option.highlightedCell,
                                        isSelected: defaultView == option.tag
                                    )
                                }
                                .buttonStyle(.plain)
                                .frame(maxWidth: .infinity)
                            }
                        }
                    }
                }

                Section {
                    VStack(alignment: .leading, spacing: 10) {
                        Text("Triple Click")
                            .font(.subheadline)
                        HStack(spacing: 8) {
                            tripleClickOption(tag: 0) {
                                HStack(spacing: 4) {
                                    Image(systemName: "arrow.up.left.and.arrow.down.right")
                                    Text("fullscreen")
                                    Text("↔")
                                        .foregroundColor(.secondary)
                                    Image(systemName: "arrow.down.right.and.arrow.up.left")
                                    Text("grid")
                                }
                                .font(.system(size: 12))
                            }
                            tripleClickOption(tag: 1) {
                                HStack(spacing: 4) {
                                    Image(systemName: "lock")
                                    Text("lock")
                                    Text("↔")
                                        .foregroundColor(.secondary)
                                    Image(systemName: "lock.open")
                                    Text("unlock")
                                }
                                .font(.system(size: 12))
                            }
                        }
                    }
                }

                Section {
                    VStack(alignment: .leading, spacing: 10) {
                        Text("Default Lock State")
                            .font(.subheadline)
                        HStack {
                            ForEach(
                                [(true, "Locked", "lock.fill"), (false, "Unlocked", "lock.open")],
                                id: \.0
                            ) { locked, label, icon in
                                Button(action: { defaultLocked = locked }) {
                                    VStack(spacing: 4) {
                                        Image(systemName: icon)
                                            .font(.system(size: 18))
                                        Text(label)
                                            .font(.caption2)
                                    }
                                    .frame(maxWidth: .infinity)
                                    .padding(.vertical, 8)
                                    .background(
                                        RoundedRectangle(cornerRadius: 8)
                                            .fill(defaultLocked == locked ? Color.blue.opacity(0.2) : Color.clear)
                                    )
                                    .overlay(
                                        RoundedRectangle(cornerRadius: 8)
                                            .stroke(defaultLocked == locked ? Color.blue : Color.gray.opacity(0.3), lineWidth: defaultLocked == locked ? 1.5 : 1)
                                    )
                                    .foregroundColor(defaultLocked == locked ? .blue : .gray)
                                }
                                .buttonStyle(.plain)
                                .accessibilityLabel("\(label), \(defaultLocked == locked ? "selected" : "not selected")")
                            }
                        }
                    }
                }

                #if os(iOS)
                Section {
                    VStack(alignment: .leading, spacing: 10) {
                        HStack {
                            Text("Caffeine Mode")
                                .font(.subheadline)
                            Button {
                                showCaffeineInfo.toggle()
                            } label: {
                                Image(systemName: "info.circle")
                                    .font(.subheadline)
                                    .foregroundStyle(.secondary)
                            }
                            .buttonStyle(.plain)
                            .accessibilityLabel("About Caffeine Mode")
                            .popover(isPresented: $showCaffeineInfo) {
                                VStack(alignment: .leading, spacing: 4) {
                                    Text("**Half** — Screen stays on. Simulations pause when you leave the app.")
                                    Text("**Full** — Screen stays on. Requests background time so simulations may continue when you leave the app.")
                                }
                                .fixedSize(horizontal: false, vertical: true)
                                .font(.caption)
                                .foregroundStyle(.secondary)
                                .frame(width: 260)
                                .padding()
                                .presentationCompactAdaptation(.popover)
                            }
                        }
                        HStack {
                            ForEach(
                                [(0, "Off", "cup.and.saucer"), (1, "Half", "cup.and.saucer.fill"), (2, "Full", "cup.and.heat.waves.fill")],
                                id: \.0
                            ) { tag, label, icon in
                                Button(action: { caffeineMode = tag }) {
                                    VStack(spacing: 4) {
                                        Image(systemName: icon)
                                            .font(.system(size: 18))
                                        Text(label)
                                            .font(.caption2)
                                    }
                                    .frame(maxWidth: .infinity)
                                    .padding(.vertical, 8)
                                    .background(
                                        RoundedRectangle(cornerRadius: 8)
                                            .fill(caffeineMode == tag ? Color.blue.opacity(0.2) : Color.clear)
                                    )
                                    .overlay(
                                        RoundedRectangle(cornerRadius: 8)
                                            .stroke(caffeineMode == tag ? Color.blue : Color.gray.opacity(0.3), lineWidth: caffeineMode == tag ? 1.5 : 1)
                                    )
                                    .foregroundColor(caffeineMode == tag ? .blue : .gray)
                                }
                                .buttonStyle(.plain)
                                .accessibilityLabel("\(label), \(caffeineMode == tag ? "selected" : "not selected")")
                            }
                        }
                    }
                }
                #endif

                Section {
                    VStack(alignment: .leading, spacing: 10) {
                        HStack {
                            Text("Default Video Transport")
                                .font(.subheadline)
                            #if !os(tvOS)
                            Button {
                                showVideoTransportInfo.toggle()
                            } label: {
                                Image(systemName: "info.circle")
                                    .font(.subheadline)
                                    .foregroundStyle(.secondary)
                            }
                            .buttonStyle(.plain)
                            .accessibilityLabel("About Default Video Transport")
                            .popover(isPresented: $showVideoTransportInfo) {
                                VStack(alignment: .leading, spacing: 4) {
                                    Text("**MJPEG/HTTP** — Works with VLC, ffplay, browsers. Simpler, more reliable.")
                                    Text("**HEVC/RTSP** — Works with VLC, ffplay. Hardware-encoded H.265, very low bandwidth.")
                                    Text("**HEVC/QUIC** — Custom receiver app. Sub-0.5s latency, zero client-side buffering.")
                                }
                                .fixedSize(horizontal: false, vertical: true)
                                .font(.caption)
                                .foregroundStyle(.secondary)
                                .frame(width: 280)
                                .padding()
                                .presentationCompactAdaptation(.popover)
                            }
                            #endif
                        }
                        HStack {
                            ForEach(
                                [(0, "MJPEG/HTTP", "photo.on.rectangle"), (1, "HEVC/RTSP", "video.fill"), (2, "HEVC/QUIC", "bolt.fill")],
                                id: \.0
                            ) { tag, label, icon in
                                Button(action: { videoTransport = tag }) {
                                    VStack(spacing: 4) {
                                        Image(systemName: icon)
                                            .font(.system(size: 18))
                                        Text(label)
                                            .font(.caption2)
                                    }
                                    .frame(maxWidth: .infinity)
                                    .padding(.vertical, 8)
                                    .background(
                                        RoundedRectangle(cornerRadius: 8)
                                            .fill(videoTransport == tag ? Color.blue.opacity(0.2) : Color.clear)
                                    )
                                    .overlay(
                                        RoundedRectangle(cornerRadius: 8)
                                            .stroke(videoTransport == tag ? Color.blue : Color.gray.opacity(0.3), lineWidth: videoTransport == tag ? 1.5 : 1)
                                    )
                                    .foregroundColor(videoTransport == tag ? .blue : .gray)
                                }
                                .buttonStyle(.plain)
                                .accessibilityLabel("\(label), \(videoTransport == tag ? "selected" : "not selected")")
                            }
                        }
                    }
                }

                #if !os(tvOS)
                Section("Network") {
                    HStack {
                        Text("Bind Address")
                            .font(.subheadline)
                        Spacer()
                        Text(bindAddress.isEmpty ? "Auto" : bindAddress)
                            .font(.system(size: 13, design: .monospaced))
                            .foregroundStyle(.secondary)
                    }

                    portRow(label: "gRPC Port", value: $grpcPort, defaultValue: 8999,
                            infoPresented: $showGrpcPortInfo,
                            infoText: "Remote simulation control channel. Changing this port stops the server and restarts it on the new port. All active simulations are re-registered automatically.")

                    ForEach(1...4, id: \.self) { inst in
                        DisclosureGroup {
                            portRow(label: "UDP Port", value: udpPortBinding(for: inst), defaultValue: 9000 + inst,
                                    infoPresented: $showUdpPortInfo,
                                    infoText: "Control (input to MuJoCo) and state (output from MuJoCo) channel for the external iMuJoCo driver. Changing this port pauses the simulation, recreates the runtime on the new port, and reloads the model automatically.")
                            portRow(label: "Camera Port", value: camPortBinding(for: inst), defaultValue: 9000 + inst * 100,
                                    infoPresented: $showCamPortInfo,
                                    infoText: "Video streaming port (MJPEG, RTP/RTSP, or HEVC/QUIC). Changing this port restarts the video streamers — physics keeps running uninterrupted.")
                        } label: {
                            LayoutIcon(highlightedCell: inst - 1, isSelected: true, size: 20)
                        }
                        .font(.subheadline)
                    }

                }
                #endif

                Section {
                    Divider()
                        .listRowInsets(EdgeInsets())
                        .listRowBackground(Color.clear)
                }

                Section {
                    Toggle("Performance Stats Bar", isOn: $showStatsBar)
                        .font(.subheadline)
                    NavigationLink {
                        AboutView()
                    } label: {
                        Text("About iMuJoCo")
                    }
                    #if !os(tvOS)
                    .listRowSeparator(.visible, edges: .all)
                    .alignmentGuide(.listRowSeparatorLeading) { _ in 0 }
                    #endif
                }
            }
            .listStyle(.plain)
            .navigationTitle("Settings")
            #if os(iOS)
            .navigationBarTitleDisplayMode(.inline)
            #endif
            .alert("Invalid Port", isPresented: $showPortWarning) {
                Button("OK", role: .cancel) {}
            } message: {
                Text(portWarningMessage)
            }
            .sheet(item: $editingPort) { target in
                PortNumPadView(
                    label: target.label,
                    currentValue: target.value.wrappedValue,
                    defaultValue: target.defaultValue,
                    registeredPorts: allRegisteredPorts
                ) { newValue in
                    let oldValue = target.value.wrappedValue
                    target.value.wrappedValue = newValue
                    applyPortChange(label: target.label, oldValue: oldValue, newValue: newValue)
                } onWarning: { msg in
                    portWarningMessage = msg
                    showPortWarning = true
                }
            }
        }
        #if os(iOS)
        .presentationDetents([.medium, .large], selection: .constant(.large))
        .presentationDragIndicator(.visible)
        #endif
        #if os(macOS)
        .frame(minWidth: 450, minHeight: 200)
        #endif
    }

    // MARK: - Port Helpers

    private var allRegisteredPorts: Set<Int> {
        [grpcPort,
         inst1UdpPort, inst2UdpPort, inst3UdpPort, inst4UdpPort,
         inst1CamPort, inst2CamPort, inst3CamPort, inst4CamPort]
    }

    private func portRow(label: String, value: Binding<Int>, defaultValue: Int,
                         infoPresented: Binding<Bool>? = nil, infoText: String? = nil) -> some View {
        HStack {
            Text(label)
                .font(.subheadline)
                .foregroundStyle(.primary)
                .onTapGesture {
                    editingPort = PortEditTarget(label: label, value: value, defaultValue: defaultValue)
                }
            #if !os(tvOS)
            if let infoPresented, let infoText {
                Button {
                    infoPresented.wrappedValue.toggle()
                } label: {
                    Image(systemName: "info.circle")
                        .font(.caption)
                        .padding(.horizontal, 4)
                        .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
                .foregroundStyle(.primary)
                .popover(isPresented: infoPresented) {
                    ScrollView {
                        Text(infoText)
                            .font(.caption)
                            .fixedSize(horizontal: false, vertical: true)
                            .padding()
                    }
                    .frame(width: 260, height: 120)
                }
            }
            #endif
            Spacer()
            Text(String(value.wrappedValue))
                .font(.system(size: 13, design: .monospaced))
                .foregroundStyle(.secondary)
                .onTapGesture {
                    editingPort = PortEditTarget(label: label, value: value, defaultValue: defaultValue)
                }
        }
    }

    private func udpPortBinding(for inst: Int) -> Binding<Int> {
        switch inst {
        case 1: return $inst1UdpPort
        case 2: return $inst2UdpPort
        case 3: return $inst3UdpPort
        case 4: return $inst4UdpPort
        default: return .constant(0)
        }
    }

    private func camPortBinding(for inst: Int) -> Binding<Int> {
        switch inst {
        case 1: return $inst1CamPort
        case 2: return $inst2CamPort
        case 3: return $inst3CamPort
        case 4: return $inst4CamPort
        default: return .constant(0)
        }
    }

    // MARK: - Live Port Rebinding

    private func applyPortChange(label: String, oldValue: Int, newValue: Int) {
        guard oldValue != newValue, let manager = gridManager else { return }

        if label == "gRPC Port" {
            manager.rebindGRPCPort(UInt16(newValue))
        } else if label == "UDP Port" {
            if let inst = instanceIndex(forUdpPort: newValue) {
                Task { @MainActor in await manager.rebindInstanceUDPPort(at: inst, newPort: UInt16(newValue)) }
            }
        } else if label == "Camera Port" {
            if let inst = instanceIndex(forCamPort: newValue) {
                manager.instance(at: inst)?.rebindCameraPort(UInt16(newValue))
            }
        }
    }

    private func applyAllPortDefaults() {
        guard let manager = gridManager else { return }
        manager.rebindGRPCPort(8999)
        for i in 0..<4 {
            let inst = i + 1
            Task { @MainActor in await manager.rebindInstanceUDPPort(at: i, newPort: UInt16(9000 + inst)) }
            manager.instance(at: i)?.rebindCameraPort(UInt16(9000 + inst * 100))
        }
    }

    private func instanceIndex(forUdpPort port: Int) -> Int? {
        if port == inst1UdpPort { return 0 }
        if port == inst2UdpPort { return 1 }
        if port == inst3UdpPort { return 2 }
        if port == inst4UdpPort { return 3 }
        return nil
    }

    private func instanceIndex(forCamPort port: Int) -> Int? {
        if port == inst1CamPort { return 0 }
        if port == inst2CamPort { return 1 }
        if port == inst3CamPort { return 2 }
        if port == inst4CamPort { return 3 }
        return nil
    }
}

// MARK: - Port Number Pad

/// Identifies which port field is being edited.
struct PortEditTarget: Identifiable {
    let id = UUID()
    let label: String
    let value: Binding<Int>
    let defaultValue: Int
}

/// Passcode-style number pad for entering a port number.
struct PortNumPadView: View {
    let label: String
    let currentValue: Int
    let defaultValue: Int
    let registeredPorts: Set<Int>
    let onSave: (Int) -> Void
    let onWarning: (String) -> Void

    @Environment(\.dismiss) private var dismiss
    @State private var digits: String

    private static let validPortRange = 1024...65535

    private let columns = Array(repeating: GridItem(.flexible(), spacing: 10), count: 3)

    init(label: String, currentValue: Int, defaultValue: Int,
         registeredPorts: Set<Int>,
         onSave: @escaping (Int) -> Void, onWarning: @escaping (String) -> Void) {
        self.label = label
        self.currentValue = currentValue
        self.defaultValue = defaultValue
        self.registeredPorts = registeredPorts
        self.onSave = onSave
        self.onWarning = onWarning
        self._digits = State(initialValue: String(currentValue))
    }

    private var isValidPort: Bool {
        guard let port = Int(digits), !digits.isEmpty else { return false }
        if !Self.validPortRange.contains(port) { return false }
        // Same as current value is always valid
        if port == currentValue { return true }
        // Conflict with another registered port
        if registeredPorts.contains(port) { return false }
        return true
    }

    private var portStatus: String? {
        if digits.isEmpty { return "Enter a port number" }
        guard let port = Int(digits) else { return "Enter a port number" }
        if !Self.validPortRange.contains(port) { return "Out of range (1024–65535)" }
        if port != currentValue && registeredPorts.contains(port) { return "Conflicts with another port" }
        return nil
    }

    var body: some View {
        VStack(spacing: 4) {
            // Title
            Text(label)
                .font(.headline)
                .padding(.top, 48)

            // Digit display — red when invalid
            Text(digits.isEmpty ? "" : digits)
                .font(.system(size: 34, weight: .medium, design: .monospaced))
                .foregroundColor(digits.isEmpty || isValidPort ? nil : .red)
                .frame(height: 40)

            // Number pad grid
            LazyVGrid(columns: columns, spacing: 10) {
                ForEach(1...9, id: \.self) { n in
                    numButton(String(n))
                }
                // Bottom row: Cancel, 0, Delete
                cancelButton()
                numButton("0")
                deleteButton()
            }
            .padding(.horizontal, 36)

            // Enter button — shows error status when port is invalid
            Button {
                commitValue()
            } label: {
                Text(portStatus ?? "Enter")
                    .font(.title3.weight(.semibold))
                    .frame(maxWidth: .infinity)
                    .frame(height: 48)
                    .background(isValidPort ? AnyShapeStyle(.tint) : AnyShapeStyle(.fill.tertiary),
                                in: RoundedRectangle(cornerRadius: 10))
                    .foregroundColor(isValidPort ? .white : .red)
            }
            .buttonStyle(.plain)
            .disabled(!isValidPort)
            .padding(.horizontal, 36)
            .padding(.top, 6)  // 6 + VStack spacing 4 = 10, same as grid row spacing
            .padding(.bottom, 48)
        }
        #if os(iOS)
        .presentationDetents([.medium])
        .presentationDragIndicator(.hidden)
        .presentationBackgroundInteraction(.disabled)
        #endif
        #if os(macOS)
        .frame(width: 300, height: 420)
        #endif
    }

    private func numButton(_ digit: String) -> some View {
        Button {
            if digits.count < 5 {
                digits.append(digit)
            }
        } label: {
            Text(digit)
                .font(.title2)
                .frame(maxWidth: .infinity)
                .frame(height: 48)
                .background(.fill.tertiary, in: RoundedRectangle(cornerRadius: 10))
        }
        .buttonStyle(.plain)
    }

    private func deleteButton() -> some View {
        Button {
            if !digits.isEmpty {
                digits.removeLast()
            }
        } label: {
            Image(systemName: "delete.backward")
                .font(.title3)
                .frame(maxWidth: .infinity)
                .frame(height: 48)
        }
        .buttonStyle(.plain)
        .disabled(digits.isEmpty)
    }

    private func cancelButton() -> some View {
        Button {
            dismiss()
        } label: {
            Text("Cancel")
                .font(.subheadline)
                .frame(maxWidth: .infinity)
                .frame(height: 48)
        }
        .buttonStyle(.plain)
    }

    private func commitValue() {
        guard let port = Int(digits), !digits.isEmpty, isValidPort else { return }
        if port == currentValue { dismiss(); return }
        if !Self.isPortAvailable(UInt16(port)) {
            dismiss()
            onWarning("Port \(port) is already in use by another process.")
            return
        }
        onSave(port)
        dismiss()
    }

    private static func isPortAvailable(_ port: UInt16) -> Bool {
        let fd = Darwin.socket(AF_INET, SOCK_STREAM, 0)
        guard fd >= 0 else { return true }
        defer { close(fd) }
        var opt: Int32 = 1
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, socklen_t(MemoryLayout<Int32>.size))
        var addr = sockaddr_in()
        addr.sin_len = UInt8(MemoryLayout<sockaddr_in>.size)
        addr.sin_family = sa_family_t(AF_INET)
        addr.sin_port = port.bigEndian
        addr.sin_addr.s_addr = INADDR_LOOPBACK.bigEndian
        let result = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                bind(fd, $0, socklen_t(MemoryLayout<sockaddr_in>.size))
            }
        }
        return result == 0
    }
}

// MARK: - About View

struct AboutView: View {
    @Environment(\.dismiss) private var dismiss

    private var appVersion: String {
        Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "–"
    }

    private var buildNumber: String {
        Bundle.main.infoDictionary?["CFBundleVersion"] as? String ?? "–"
    }

    private let mujocoVersion = "3.4.0"

    private let modelInfo: [(name: String, source: String, license: String)] = [
        ("Car, Humanoid (Supine)", "MuJoCo", "Apache 2.0"),
        ("Simple Pendulum", "iMuJoCo", "Apache 2.0"),
        ("Agility Cassie, ANYmal C, Unitree G1, Unitree H1", "Menagerie", "BSD-3"),
    ]

    var body: some View {
        List {
            Section("App") {
                HStack {
                    Text("Version")
                    Spacer()
                    Text("\(appVersion) (\(buildNumber))")
                        .foregroundStyle(.secondary)
                }
                HStack {
                    Text("MuJoCo")
                    Spacer()
                    Text(mujocoVersion)
                        .foregroundStyle(.secondary)
                }
                HStack {
                    Text("License")
                    Spacer()
                    Text("Apache 2.0")
                        .foregroundStyle(.secondary)
                }
                HStack {
                    Text("App Icon")
                    Spacer()
                    Text("crabe.art")
                        .foregroundStyle(.secondary)
                }
            }

            Section("Bundled Models") {
                ForEach(modelInfo, id: \.name) { model in
                    VStack(alignment: .leading, spacing: 2) {
                        Text(model.name)
                        Text("\(model.source) · \(model.license)")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
            }

        }
        .navigationTitle("About")
        #if os(iOS)
        .navigationBarTitleDisplayMode(.inline)
        #endif
        .navigationBarBackButtonHidden(true)
        .toolbar {
            ToolbarItem(placement: .navigation) {
                Button(action: { dismiss() }) {
                    Image(systemName: "chevron.left")
                        .font(.system(size: 14, weight: .medium))
                }
            }
        }
    }
}

// MARK: - Preview

#if DEBUG
#Preview {
    SimulationGridView(gridManager: SimulationGridManager())
}
#endif
