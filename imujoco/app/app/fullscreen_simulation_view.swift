// fullscreen_simulation_view.swift
// Fullscreen view for single simulation with controls

import SwiftUI
import render

struct FullscreenSimulationView: View {
    var instance: SimulationInstance
    var onExit: () -> Void

    @State private var showMetrics = true

    var body: some View {
        ZStack {
            // Metal rendering view (full screen)
            MuJoCoMetalView(dataSource: instance)
                .ignoresSafeArea()

            // Controls overlay (always visible)
            controlsOverlay
        }
        .background(Color.black)
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
        VStack {
            // Top HUD
            topHUD
                .padding(.top, 8)

            Spacer()

            // Bottom-right: port + status + time
            HStack {
                Spacer()
                VStack(alignment: .trailing, spacing: 3) {
                    Text(verbatim: ":\(instance.port)")
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
                .onTapGesture(count: 3) {
                    withAnimation(.easeInOut(duration: 0.3)) {
                        onExit()
                    }
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
        onExit: {}
    )
}
#endif
