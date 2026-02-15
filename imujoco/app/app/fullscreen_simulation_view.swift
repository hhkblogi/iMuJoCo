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

            // Bottom: status/time (right) + controls (center)
            HStack(alignment: .bottom) {
                Spacer()

                bottomControls

                Spacer()

                // Status + time (always visible, bottom-right)
                VStack(alignment: .trailing, spacing: 3) {
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
        HStack {
            #if os(iOS)
            // Back button
            Button(action: onExit) {
                Image(systemName: "chevron.left")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(overlayTextColor(brightness: brightness))
                    .frame(width: 44, height: 44)
                    .background(Color.black.opacity(0.3))
                    .clipShape(Circle())
            }
            #endif

            #if os(macOS)
            // Back button for macOS
            Button(action: onExit) {
                Image(systemName: "chevron.left")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(overlayTextColor(brightness: brightness))
                    .frame(width: 44, height: 44)
                    .background(Color.black.opacity(0.3))
                    .clipShape(Circle())
            }
            .buttonStyle(.plain)
            #endif

            Spacer()

            // Model info
            VStack(spacing: 4) {
                Text(instance.modelName)
                    .font(.headline)
                    .foregroundColor(overlayTextColor(brightness: brightness))

                Text(verbatim: "Port \(instance.port)")
                    .font(.caption)
                    .foregroundColor(overlaySecondaryTextColor(brightness: brightness))
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 8)
            .background(Color.black.opacity(0.3))
            .clipShape(Capsule())

            Spacer()

            // Performance metrics (toggleable)
            if showMetrics {
                VStack(alignment: .trailing, spacing: 4) {
                    // Performance metrics
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
                .padding(.horizontal, 12)
                .padding(.vertical, 8)
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

    private var bottomControls: some View {
        HStack(spacing: 24) {
            #if os(iOS)
            // Metrics toggle
            ControlButton(
                systemName: showMetrics ? "chart.bar.fill" : "chart.bar",
                action: {
                    withAnimation(.easeInOut(duration: 0.2)) {
                        showMetrics.toggle()
                    }
                }
            )

            // Reset button
            ControlButton(systemName: "arrow.counterclockwise", action: instance.reset)

            // Step button
            ControlButton(systemName: "forward.frame.fill", action: instance.step)
                .disabled(instance.state == .running)

            // Play/Pause button
            ControlButton(
                systemName: instance.state == .running ? "pause.fill" : "play.fill",
                action: instance.togglePlayPause
            )
            .frame(width: 64, height: 64)
            #endif

            #if os(macOS)
            // Metrics toggle
            Button(action: {
                withAnimation(.easeInOut(duration: 0.2)) {
                    showMetrics.toggle()
                }
            }) {
                Image(systemName: showMetrics ? "chart.bar.fill" : "chart.bar")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                    .frame(width: 48, height: 48)
                    .background(Color.white.opacity(0.2))
                    .clipShape(Circle())
            }
            .buttonStyle(.plain)

            // Reset button
            Button(action: instance.reset) {
                Image(systemName: "arrow.counterclockwise")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                    .frame(width: 48, height: 48)
                    .background(Color.white.opacity(0.2))
                    .clipShape(Circle())
            }
            .buttonStyle(.plain)

            // Step button
            Button(action: instance.step) {
                Image(systemName: "forward.frame.fill")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                    .frame(width: 48, height: 48)
                    .background(Color.white.opacity(0.2))
                    .clipShape(Circle())
            }
            .buttonStyle(.plain)
            .disabled(instance.state == .running)

            // Play/Pause button
            Button(action: instance.togglePlayPause) {
                Image(systemName: instance.state == .running ? "pause.fill" : "play.fill")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                    .frame(width: 64, height: 64)
                    .background(Color.white.opacity(0.2))
                    .clipShape(Circle())
            }
            .buttonStyle(.plain)
            #endif

            #if os(tvOS)
            // tvOS hint
            Text("Swipe: Rotate | Up/Down: Zoom | Select: Play/Pause | Menu: Back")
                .font(.caption)
                .foregroundColor(.white.opacity(0.7))
            #endif
        }
        .padding(.horizontal, 24)
        .padding(.vertical, 16)
        .background(Color.black.opacity(0.5))
        .clipShape(Capsule())
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
