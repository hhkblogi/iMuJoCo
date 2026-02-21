// fullscreen_simulation_view.swift
// Fullscreen view for single simulation with controls

import SwiftUI
import render

struct FullscreenSimulationView: View {
    var instance: SimulationInstance
    var instanceIndex: Int
    var onExit: () -> Void
    var onSwitchInstance: (Int) -> Void  // -1 = grid, 0-3 = instance
    var onLoadModel: () -> Void

    @AppStorage("tripleClickAction") private var tripleClickAction: Int = 0
    @State private var showMetrics = true
    @State private var resetProgress: CGFloat = 0
    @State private var stopProgress: CGFloat = 0
    @State private var isNavigating = false
    @State private var showPortInfo = false
    @State private var showCamInfo = false
    @State private var hoveredCell: Int? = nil
    @State private var navGridFrame: CGRect = .zero

    var body: some View {
        ZStack {
            if instance.isLoading {
                // Loading indicator while model is being loaded
                fullscreenLoadingView
            } else if instance.isActive {
                // Metal rendering view — hidden when blinded to save GPU
                if instance.isBlinded {
                    Color.black.ignoresSafeArea()
                    Image(systemName: "eye.slash.fill")
                        .font(.system(size: 40))
                        .foregroundColor(.gray.opacity(0.3))
                } else {
                    MuJoCoMetalView(dataSource: instance, isFullscreen: true)
                        .allowsHitTesting(!instance.isLocked)
                        .ignoresSafeArea()
                }

                // Controls overlay (always visible)
                controlsOverlay
            } else {
                // Empty view — model not loaded
                fullscreenEmptyView
            }

            // Navigation grid overlay (press on layout icon)
            if isNavigating {
                VStack {
                    Spacer()
                    navGrid
                        .background(GeometryReader { geo in
                            Color.clear.preference(key: NavGridFrameKey.self, value: geo.frame(in: .global))
                        })
                        .padding(.bottom, 40)
                }
                .allowsHitTesting(false)
                .transition(.opacity)
                .onPreferenceChange(NavGridFrameKey.self) { navGridFrame = $0 }
            }

            // Bottom-center: layout icon with navigation gesture
            VStack {
                Spacer()
                LayoutIcon(
                    highlightedCell: instanceIndex,
                    isSelected: true,
                    size: 20,
                    tintColor: overlaySecondaryTextColor(brightness: brightness)
                )
                .opacity(isNavigating ? 0 : 1)
                #if !os(tvOS)
                .gesture(
                    DragGesture(minimumDistance: 0, coordinateSpace: .global)
                        .onChanged { value in
                            if !isNavigating {
                                withAnimation(.easeOut(duration: 0.2)) {
                                    isNavigating = true
                                    hoveredCell = instanceIndex
                                }
                            } else {
                                hoveredCell = navCellAt(value.location)
                            }
                        }
                        .onEnded { _ in
                            if isNavigating, let cell = hoveredCell {
                                if cell == -1 {
                                    onExit()  // Center zone → grid view
                                } else if cell != instanceIndex {
                                    onSwitchInstance(cell)
                                }
                            }
                            withAnimation(.easeOut(duration: 0.15)) {
                                isNavigating = false
                            }
                            hoveredCell = nil
                        }
                )
                #endif
                .padding(.bottom, 12)
            }
        }
        .background(Color.black)
        .contentShape(Rectangle())
        .onTripleTap(dotColor: instance.isActive ? overlayTextColor(brightness: brightness) : .gray, targetLabel: tripleClickAction == 0 ? "grid view" : (instance.isActive ? "lock/unlock" : "")) {
            if tripleClickAction == 0 {
                withAnimation(.easeInOut(duration: 0.3)) {
                    onExit()
                }
            } else if instance.isActive {
                instance.isLocked.toggle()
            }
        }
        #if os(iOS)
        .statusBarHidden(true)
        .persistentSystemOverlays(.hidden)
        #endif
        #if os(tvOS)
        .onPlayPauseCommand {
            instance.togglePlayPause()
        }
        .onExitCommand {
            onExit()
        }
        #endif
    }

    // MARK: - Controls Overlay

    private var controlsOverlay: some View {
        ZStack {
            // Large centered countdown overlays (always present so trim animates)
            countdownOverlay(progress: resetProgress, systemImage: "arrow.counterclockwise", color: .orange, iconSize: 40, ringWidth: 6)
            countdownOverlay(progress: stopProgress, systemImage: "stop.fill", color: .red, iconSize: 40, ringWidth: 6)

            // Left control bar
            VStack {
                HStack {
                    VStack(spacing: 8) {
                        // Lock button (standalone toggle)
                        Button(action: { instance.isLocked.toggle() }) {
                            let frameSize: CGFloat = 14 * 2.2
                            Image(systemName: instance.isLocked ? "lock.fill" : "lock.open")
                                .font(.system(size: 14, weight: .semibold))
                                .foregroundColor(overlayTextColor(brightness: brightness))
                                .frame(width: frameSize, height: frameSize)
                        }
                        .buttonStyle(.plain)

                        // Eye button (toggle rendering)
                        Button(action: { instance.isBlinded.toggle() }) {
                            let frameSize: CGFloat = 14 * 2.2
                            Image(systemName: instance.isBlinded ? "eye.slash.fill" : "eye.fill")
                                .font(.system(size: 14, weight: .semibold))
                                .foregroundColor(overlayTextColor(brightness: brightness))
                                .frame(width: frameSize, height: frameSize)
                        }
                        .buttonStyle(.plain)

                        // Controls pill (visible when unlocked)
                        if !instance.isLocked {
                            VStack(spacing: 6) {
                                Button(action: { instance.togglePlayPause() }) {
                                    let isRunning = instance.state == .running
                                    let frameSize: CGFloat = 14 * 2.2
                                    Image(systemName: isRunning ? "pause.fill" : "play.fill")
                                        .font(.system(size: 14, weight: .semibold))
                                        .foregroundColor(overlayTextColor(brightness: brightness))
                                        .frame(width: frameSize, height: frameSize)
                                }
                                .buttonStyle(.plain)
                                LongPressButton(
                                    systemImage: "arrow.counterclockwise",
                                    duration: 3.0,
                                    brightness: brightness,
                                    iconSize: 14,
                                    action: { instance.reset() },
                                    holdProgress: $resetProgress
                                )
                                LongPressButton(
                                    systemImage: "stop.fill",
                                    duration: 3.0,
                                    brightness: brightness,
                                    iconSize: 14,
                                    action: { instance.unload() },
                                    holdProgress: $stopProgress
                                )
                                if !instance.isBlinded {
                                    Button(action: { instance.resetCamera() }) {
                                        let frameSize: CGFloat = 14 * 2.2
                                        Image(systemName: "camera.metering.center.weighted")
                                            .font(.system(size: 14, weight: .semibold))
                                            .foregroundColor(overlayTextColor(brightness: brightness))
                                            .frame(width: frameSize, height: frameSize)
                                    }
                                    .buttonStyle(.plain)
                                }
                            }
                            .padding(4)
                            .background(
                                RoundedRectangle(cornerRadius: 10)
                                    .fill(Color.black.opacity(0.35))
                            )
                        }
                    }
                    Spacer()
                }
                .padding(.leading, 12)
                .padding(.top, 44)
                Spacer()
            }

            VStack {
                // Top HUD
                topHUD
                    .padding(.top, 4)

                Spacer()

                // Bottom-right: port + status + time
                HStack {
                    Spacer()
                    VStack(alignment: .trailing, spacing: 3) {
                        HStack(spacing: 3) {
                            Text(verbatim: "C/S")
                                .font(.system(size: 11, weight: .medium))
                            #if !os(tvOS)
                            Image(systemName: "info.circle")
                                .font(.system(size: 9))
                                .onTapGesture { showPortInfo = true }
                                .popover(isPresented: $showPortInfo) {
                                    VStack(alignment: .leading, spacing: 4) {
                                        Text("C/S = Control / State")
                                            .font(.system(size: 13, weight: .semibold))
                                        Text("Bidirectional UDP port for\ncontrol input and state output")
                                            .font(.system(size: 12))
                                            .foregroundColor(.secondary)
                                    }
                                    .padding(10)
                                }
                            #endif
                            Text(verbatim: ":\(instance.port)")
                                .font(.system(size: 11, weight: .medium))
                        }
                        .foregroundColor(overlaySecondaryTextColor(brightness: brightness))
                        if instance.isStreaming {
                            HStack(spacing: 3) {
                                Text(verbatim: "Cam0")
                                    .font(.system(size: 11, weight: .medium))
                                #if !os(tvOS)
                                Image(systemName: "info.circle")
                                    .font(.system(size: 9))
                                    .onTapGesture { showCamInfo = true }
                                    .popover(isPresented: $showCamInfo) {
                                        let ip = getDeviceIPAddress() ?? "<ip>"
                                        VStack(alignment: .leading, spacing: 4) {
                                            Text("Cam0 = Default Free Camera")
                                                .font(.system(size: 13, weight: .semibold))
                                            Text("Video stream port (UDP + HTTP)\nfor offscreen camera capture")
                                                .font(.system(size: 12))
                                                .foregroundColor(.secondary)
                                            Text(verbatim: "http://\(ip):\(instance.cameraPort)")
                                                .font(.system(size: 12, design: .monospaced))
                                                .foregroundColor(.accentColor)
                                                .textSelection(.enabled)
                                        }
                                        .padding(10)
                                    }
                                #endif
                                Text(verbatim: ":\(instance.cameraPort)")
                                    .font(.system(size: 11, weight: .medium))
                            }
                            .foregroundColor(overlaySecondaryTextColor(brightness: brightness))
                        }
                        HStack(spacing: 4) {
                            Circle()
                                .fill(instance.state == .running ? Color.green : Color.yellow)
                                .frame(width: 8, height: 8)
                            Text(instance.stateDescription)
                                .font(.caption)
                                .foregroundColor(overlaySecondaryTextColor(brightness: brightness))
                        }
                        Text(formatSimulationTime(instance.simulationTime))
                            .font(.system(size: 11, weight: .medium, design: .monospaced))
                            .foregroundColor(overlayTextColor(brightness: brightness).opacity(0.8))
                    }
                }
                .padding(.bottom, 8)
            }
            .padding(.horizontal)
        }
    }

    private var brightness: Float { instance.sceneBrightness }

    private var metricLabelColor: Color {
        overlayTertiaryTextColor(brightness: brightness)
    }

    private var topHUD: some View {
        VStack(alignment: .leading, spacing: 4) {
            // Title
            HStack {
                Text(instance.modelName)
                    .font(.headline)
                    .foregroundColor(overlayTextColor(brightness: brightness))
                    .lineLimit(1)
                Spacer()
            }

            // Metrics row (right-aligned, under title)
            if showMetrics {
                HStack {
                    Spacer()
                    Grid(horizontalSpacing: 3, verticalSpacing: 2) {
                        GridRow {
                            Text(String(format: "%7.1f", instance.stepsPerSecondFloat))
                                .font(.system(size: 11, weight: .bold, design: .monospaced))
                                .foregroundColor(stepsPerSecondColor)
                                .gridColumnAlignment(.trailing)
                            Text("fps")
                                .font(.system(size: 9, weight: .medium))
                                .foregroundColor(metricLabelColor)
                                .gridColumnAlignment(.leading)
                            Text("SIM")
                                .font(.system(size: 9, weight: .medium))
                                .foregroundColor(metricLabelColor)
                                .gridColumnAlignment(.leading)
                        }
                        if instance.hasClient || instance.txRate > 0 {
                            GridRow {
                                Text(String(format: "%7.1f", instance.txRate))
                                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                                    .foregroundColor(rateColor(instance.txRate))
                                Text("fps")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(metricLabelColor)
                                Text("State TX")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(metricLabelColor)
                            }
                        }
                        if instance.hasClient || instance.rxRate > 0 {
                            GridRow {
                                Text(String(format: "%7.1f", instance.rxRate))
                                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                                    .foregroundColor(rateColor(instance.rxRate))
                                Text("fps")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(metricLabelColor)
                                Text("Control RX")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(metricLabelColor)
                            }
                        }
                        if instance.isStreaming {
                            GridRow {
                                Text(String(format: "%7.1f", instance.videoFPS))
                                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                                    .foregroundColor(rateColor(Float(instance.videoFPS)))
                                Text("fps")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(metricLabelColor)
                                Text("Cam0")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(metricLabelColor)
                            }
                        }
                    }
                    .fixedSize()
                }
                .transition(.opacity)
            }
        }
    }

    private var stepsPerSecondColor: Color {
        let sps = instance.stepsPerSecondFloat
        if sps >= 400 {
            return .green
        } else if sps >= 200 {
            return .yellow
        } else if sps >= 100 {
            return .orange
        } else {
            return .red
        }
    }

    // MARK: - Navigation Grid

    private var navGrid: some View {
        let gridSize: CGFloat = 140
        let gap: CGFloat = 4
        let cellSize = (gridSize - gap) / 2

        return VStack(spacing: gap) {
            HStack(spacing: gap) {
                navCell(0, size: cellSize)
                navCell(1, size: cellSize)
            }
            HStack(spacing: gap) {
                navCell(2, size: cellSize)
                navCell(3, size: cellSize)
            }
        }
        .padding(8)
        .background(
            RoundedRectangle(cornerRadius: 12)
                .fill(Color.black.opacity(0.7))
        )
    }

    @ViewBuilder
    private func navCell(_ index: Int, size: CGFloat) -> some View {
        let isCurrent = index == instanceIndex
        let isHovered = hoveredCell == index || hoveredCell == -1  // -1 = all cells (grid view)

        RoundedRectangle(cornerRadius: 4)
            .fill(isHovered ? Color.white.opacity(0.4) : (isCurrent ? Color.white.opacity(0.2) : Color.gray.opacity(0.15)))
            .overlay(
                RoundedRectangle(cornerRadius: 4)
                    .stroke(isHovered ? Color.white.opacity(0.8) : Color.gray.opacity(0.3), lineWidth: isHovered ? 2 : 1)
            )
            .frame(width: size, height: size)
    }

    private func navCellAt(_ point: CGPoint) -> Int? {
        guard navGridFrame.width > 0 else { return nil }
        let padding: CGFloat = 8
        let innerFrame = navGridFrame.insetBy(dx: padding, dy: padding)
        let relX = point.x - innerFrame.minX
        let relY = point.y - innerFrame.minY

        guard relX >= 0, relX <= innerFrame.width, relY >= 0, relY <= innerFrame.height else {
            return nil  // Outside grid — no selection
        }

        // Center zone: near the intersection of all 4 cells → grid view (-1)
        let centerX = innerFrame.width / 2
        let centerY = innerFrame.height / 2
        let centerRadius: CGFloat = 18
        if abs(relX - centerX) < centerRadius && abs(relY - centerY) < centerRadius {
            return -1  // Grid view
        }

        let col = relX < centerX ? 0 : 1
        let row = relY < centerY ? 0 : 1
        return row * 2 + col
    }

    // MARK: - Loading View

    private var fullscreenLoadingView: some View {
        VStack(spacing: 20) {
            ProgressView()
                .controlSize(.large)
                .tint(.white)
            Text(instance.loadingModelName)
                .font(.title3)
                .fontWeight(.semibold)
                .foregroundColor(.white)
                .lineLimit(1)
            Text("Loading...")
                .font(.subheadline)
                .foregroundColor(.gray)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    // MARK: - Empty View (no model loaded)

    private var fullscreenEmptyView: some View {
        VStack(spacing: 24) {
            Image(systemName: "cube.transparent")
                .font(.system(size: 60))
                .foregroundColor(.gray.opacity(0.5))

            Button(action: onLoadModel) {
                Label("Load Model", systemImage: "plus.circle.fill")
                    .font(.headline)
            }
            .buttonStyle(.bordered)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
}

// MARK: - Preference Key for Navigation Grid Frame

private struct NavGridFrameKey: PreferenceKey {
    static var defaultValue: CGRect = .zero
    static func reduce(value: inout CGRect, nextValue: () -> CGRect) {
        value = nextValue()
    }
}

// MARK: - Control Button

#if os(iOS)
struct ControlButton: View {
    let systemName: String
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            Image(systemName: systemName)
                .font(.title2)
                .fontWeight(.semibold)
                .foregroundColor(.white)
                .frame(width: 48, height: 48)
                .background(Color.white.opacity(0.2))
                .clipShape(Circle())
        }
    }
}
#endif

// MARK: - Preview

#if DEBUG
#Preview {
    FullscreenSimulationView(
        instance: SimulationInstance(id: 0),
        instanceIndex: 0,
        onExit: {},
        onSwitchInstance: { _ in },
        onLoadModel: {}
    )
}
#endif
