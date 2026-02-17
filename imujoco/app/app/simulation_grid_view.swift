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
    @State private var ipExpanded = false
    @State private var showingSettings = false
    @AppStorage("defaultView") private var defaultView: Int = 0

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
                modelGroups: gridManager.bundledModelsBySource,
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
                modelGroups: gridManager.bundledModelsBySource,
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
        .sheet(isPresented: $showingSettings) {
            SettingsView(defaultView: $defaultView, onDismiss: { showingSettings = false })
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

    private var ipIconColor: Color {
        deviceIP == "No Network" ? .red : .green
    }

    private var menuBar: some View {
        HStack {
            // Settings button
            Button(action: { showingSettings = true }) {
                Image(systemName: "slider.horizontal.3")
                    .font(.system(size: 18, weight: .medium))
                    .foregroundColor(.white)
            }
            .buttonStyle(.plain)

            Spacer()

            // Collapsible IP address capsule
            HStack(spacing: ipExpanded ? 8 : 0) {
                Image(systemName: "network")
                    .foregroundColor(ipIconColor)

                if ipExpanded {
                    Text(deviceIP)
                        .font(.system(size: 11, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .transition(.opacity.combined(with: .move(edge: .trailing)))
                }
            }
            .padding(.horizontal, ipExpanded ? 12 : 8)
            .padding(.vertical, 6)
            .background(Color.white.opacity(0.1))
            .clipShape(Capsule())
            .onTapGesture {
                withAnimation(.easeInOut(duration: 0.25)) {
                    ipExpanded.toggle()
                }
            }
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
                        Section(header: Text(group.source.rawValue)) {
                            ForEach(group.models, id: \.name) { model in
                                Button(action: { onSelectModel(model.name) }) {
                                    HStack {
                                        Text(model.name)
                                        Spacer()
                                    }
                                    .contentShape(Rectangle())
                                }
                                .buttonStyle(.plain)
                            }
                        }
                    }
                } else {
                    ForEach(modelNames, id: \.self) { name in
                        Button(action: { onSelectModel(name) }) {
                            HStack {
                                Image(systemName: "cube.fill")
                                    .foregroundColor(.blue)
                                Text(name)
                                Spacer()
                            }
                            .contentShape(Rectangle())
                        }
                        .buttonStyle(.plain)
                    }
                }
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
    var onDismiss: () -> Void

    // tag 0 = grid, 1-4 = fullscreen instance (highlightedCell 0-3)
    private let viewOptions: [(highlightedCell: Int?, tag: Int)] = [
        (nil, 0),
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4),
    ]

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
            }
            .listStyle(.plain)
            .navigationTitle("Settings")
            #if os(iOS)
            .navigationBarTitleDisplayMode(.inline)
            #endif
        }
        #if os(iOS)
        .presentationDetents([.medium])
        #endif
        #if os(macOS)
        .frame(minWidth: 450, minHeight: 200)
        #endif
    }
}

// MARK: - Preview

#if DEBUG
#Preview {
    SimulationGridView(gridManager: SimulationGridManager())
}
#endif
