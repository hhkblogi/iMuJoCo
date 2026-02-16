// fullscreen_simulation_view.swift
// Fullscreen view for single simulation with controls

import SwiftUI
import render

struct FullscreenSimulationView: View {
    var instance: SimulationInstance
    var onExit: () -> Void
    var onLoadModel: () -> Void

    @State private var showMetrics = true
    @State private var resetProgress: CGFloat = 0
    @State private var stopProgress: CGFloat = 0
    @State private var isLocked: Bool = true

    var body: some View {
        ZStack {
            if instance.isActive {
                // Metal rendering view — hidden when blinded to save GPU
                if instance.isBlinded {
                    Color.black.ignoresSafeArea()
                    Image(systemName: "eye.slash.fill")
                        .font(.system(size: 40))
                        .foregroundColor(.gray.opacity(0.3))
                } else {
                    MuJoCoMetalView(dataSource: instance)
                        .allowsHitTesting(!isLocked)
                        .ignoresSafeArea()
                }

                // Controls overlay (always visible)
                controlsOverlay
            } else {
                // Empty view — model not loaded
                fullscreenEmptyView
            }
        }
        .background(Color.black)
        .contentShape(Rectangle())
        .onTripleTap(dotColor: instance.isActive ? overlayTextColor(brightness: brightness) : .gray, targetLabel: "grid view") {
            withAnimation(.easeInOut(duration: 0.3)) {
                onExit()
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
                        Button(action: { isLocked.toggle() }) {
                            let frameSize: CGFloat = 14 * 2.2
                            Image(systemName: isLocked ? "lock.fill" : "lock.open")
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
                        if !isLocked {
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
                                RoundedRectangle(cornerRadius: 18)
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
                    .padding(.top, 8)

                Spacer()

                // Bottom-right: port + status + time
                HStack {
                    Spacer()
                    VStack(alignment: .trailing, spacing: 3) {
                    Text(verbatim: "Port :\(instance.port)")
                        .font(.system(size: 11, weight: .medium))
                        .foregroundColor(overlaySecondaryTextColor(brightness: brightness))
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
            // Title row: model name — triple-click to exit fullscreen
            Text(instance.modelName)
                .font(.headline)
                .foregroundColor(overlayTextColor(brightness: brightness))
                .lineLimit(1)

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

    // MARK: - Empty View (no model loaded)

    private var fullscreenEmptyView: some View {
        VStack(spacing: 24) {
            Image(systemName: "cube.transparent")
                .font(.system(size: 60))
                .foregroundColor(.gray.opacity(0.5))

            Text("No Model Loaded")
                .font(.title3)
                .foregroundColor(.gray)

            Button(action: onLoadModel) {
                Label("Load Model", systemImage: "plus.circle.fill")
                    .font(.headline)
            }
            .buttonStyle(.bordered)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
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
        onExit: {},
        onLoadModel: {}
    )
}
#endif
