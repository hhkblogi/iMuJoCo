// fullscreen_simulation_view.swift
// Fullscreen view for single simulation with controls

import SwiftUI
import render

struct FullscreenSimulationView: View {
    var instance: SimulationInstance
    var onExit: () -> Void

    @State private var showControls = true
    @State private var hideControlsTask: Task<Void, Never>?

    var body: some View {
        ZStack {
            // Metal rendering view (full screen)
            MuJoCoMetalView(dataSource: instance)
                .ignoresSafeArea()

            // Controls overlay
            if showControls {
                controlsOverlay
                    .transition(.opacity)
            }
        }
        .background(Color.black)
        #if os(iOS)
        .statusBarHidden(true)
        .persistentSystemOverlays(.hidden)
        .onTapGesture {
            withAnimation(.easeInOut(duration: 0.2)) {
                showControls.toggle()
            }
            scheduleHideControls()
        }
        #endif
        #if os(tvOS)
        .onPlayPauseCommand {
            instance.togglePlayPause()
        }
        .onExitCommand {
            onExit()
        }
        #endif
        .onAppear {
            scheduleHideControls()
        }
        .onDisappear {
            hideControlsTask?.cancel()
            hideControlsTask = nil
        }
    }

    // MARK: - Controls Overlay

    private var controlsOverlay: some View {
        VStack {
            // Top HUD
            topHUD
                .padding(.top, 8)

            Spacer()

            // Bottom controls
            bottomControls
                .padding(.bottom, 8)
        }
        .padding(.horizontal)
    }

    private var topHUD: some View {
        HStack {
            #if os(iOS)
            // Back button
            Button(action: onExit) {
                Image(systemName: "chevron.left")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                    .frame(width: 44, height: 44)
                    .background(Color.black.opacity(0.5))
                    .clipShape(Circle())
            }
            #endif

            #if os(macOS)
            // Back button for macOS
            Button(action: onExit) {
                Image(systemName: "chevron.left")
                    .font(.title2)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                    .frame(width: 44, height: 44)
                    .background(Color.black.opacity(0.5))
                    .clipShape(Circle())
            }
            .buttonStyle(.plain)
            #endif

            Spacer()

            // Model info
            VStack(spacing: 4) {
                Text(instance.modelName)
                    .font(.headline)
                    .foregroundColor(.white)

                Text(verbatim: "Port \(instance.port)")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.7))
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 8)
            .background(Color.black.opacity(0.5))
            .clipShape(Capsule())

            Spacer()

            // Status, time, and performance metrics
            VStack(alignment: .trailing, spacing: 4) {
                HStack(spacing: 4) {
                    Circle()
                        .fill(instance.state == .running ? Color.green : Color.yellow)
                        .frame(width: 8, height: 8)
                    Text(instance.stateDescription)
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.8))
                }

                Text(formatSimulationTime(instance.simulationTime))
                    .font(.system(size: 11, weight: .medium, design: .monospaced))
                    .foregroundColor(.white.opacity(0.8))

                Divider()
                    .frame(width: 60)
                    .background(Color.white.opacity(0.3))

                // Performance metrics
                Grid(horizontalSpacing: 3, verticalSpacing: 2) {
                    GridRow {
                        Text(String(format: "%.1f", instance.stepsPerSecondFloat))
                            .font(.system(size: 11, weight: .bold, design: .monospaced))
                            .foregroundColor(stepsPerSecondColor)
                            .gridColumnAlignment(.trailing)
                        Text("fps")
                            .font(.system(size: 9, weight: .medium))
                            .foregroundColor(.white.opacity(0.5))
                            .gridColumnAlignment(.leading)
                        Text("SIM")
                            .font(.system(size: 9, weight: .medium))
                            .foregroundColor(.white.opacity(0.5))
                            .gridColumnAlignment(.leading)
                    }
                    if instance.hasClient || instance.txRate > 0 {
                        GridRow {
                            Text(String(format: "%.1f", instance.txRate))
                                .font(.system(size: 11, weight: .bold, design: .monospaced))
                                .foregroundColor(rateColor(instance.txRate))
                            Text("fps")
                                .font(.system(size: 9, weight: .medium))
                                .foregroundColor(.white.opacity(0.5))
                            Text("State TX")
                                .font(.system(size: 9, weight: .medium))
                                .foregroundColor(.white.opacity(0.5))
                        }
                    }
                    if instance.hasClient || instance.rxRate > 0 {
                        GridRow {
                            Text(String(format: "%.1f", instance.rxRate))
                                .font(.system(size: 11, weight: .bold, design: .monospaced))
                                .foregroundColor(rateColor(instance.rxRate))
                            Text("fps")
                                .font(.system(size: 9, weight: .medium))
                                .foregroundColor(.white.opacity(0.5))
                            Text("Control RX")
                                .font(.system(size: 9, weight: .medium))
                                .foregroundColor(.white.opacity(0.5))
                        }
                    }
                }
                .fixedSize()

            }
            .padding(.horizontal, 12)
            .padding(.vertical, 8)
            .background(Color.black.opacity(0.5))
            .clipShape(RoundedRectangle(cornerRadius: 8))
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

    // MARK: - Helpers

    private func scheduleHideControls() {
        hideControlsTask?.cancel()
        hideControlsTask = Task {
            try? await Task.sleep(nanoseconds: 3_000_000_000)  // 3 seconds
            if !Task.isCancelled {
                await MainActor.run {
                    withAnimation(.easeInOut(duration: 0.3)) {
                        showControls = false
                    }
                }
            }
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
