// simulation_cell_view.swift
// Individual cell view for simulation grid

import SwiftUI
import render

// MARK: - Shared Time Formatter

func formatSimulationTime(_ seconds: Double) -> String {
    let totalMs = Int(seconds * 1000)
    return String(format: "%02d:%02d:%02d.%03d",
                  totalMs / 3_600_000,
                  (totalMs % 3_600_000) / 60_000,
                  (totalMs % 60_000) / 1000,
                  totalMs % 1000)
}

// MARK: - Shared Overlay Text Color

/// Returns white text for dark scenes, black text for bright scenes.
/// Brightness is 0.0 (dark) to 1.0 (bright), with threshold at 0.5.
func overlayTextColor(brightness: Float) -> Color {
    brightness > 0.5 ? .black : .white
}

func overlaySecondaryTextColor(brightness: Float) -> Color {
    brightness > 0.5 ? .black.opacity(0.6) : .white.opacity(0.7)
}

func overlayTertiaryTextColor(brightness: Float) -> Color {
    brightness > 0.5 ? .black.opacity(0.4) : .white.opacity(0.5)
}

// MARK: - Triple-Tap Gesture with Visual Hints

/// Shows progress dots (● ● ○) and a countdown hint under the view as the user
/// taps toward a triple-tap. Resets after 500ms of inactivity.
private struct TripleTapModifier: ViewModifier {
    let dotColor: Color
    let targetLabel: String  // e.g. "fullscreen" or "grid"
    let action: () -> Void

    @State private var tapCount = 0
    @State private var resetTask: Task<Void, Never>?

    private var remaining: Int { 3 - tapCount }

    func body(content: Content) -> some View {
        content
            .onTapGesture {
                handleTap()
            }
    }

    private func handleTap() {
        resetTask?.cancel()

        withAnimation(.easeInOut(duration: 0.12)) {
            tapCount += 1
        }

        if tapCount >= 3 {
            withAnimation(.easeInOut(duration: 0.12)) {
                tapCount = 0
            }
            action()
            return
        }

        resetTask = Task { @MainActor in
            try? await Task.sleep(nanoseconds: 500_000_000)
            guard !Task.isCancelled else { return }
            withAnimation(.easeOut(duration: 0.2)) {
                tapCount = 0
            }
        }
    }
}

extension View {
    func onTripleTap(dotColor: Color, targetLabel: String, perform action: @escaping () -> Void) -> some View {
        modifier(TripleTapModifier(dotColor: dotColor, targetLabel: targetLabel, action: action))
    }
}

// MARK: - Long-Press Button with Countdown Ring

/// A button that requires a sustained press to activate, with a circular
/// progress ring that fills over the hold duration. Prevents accidental taps.
/// Uses DragGesture + async timer instead of onLongPressGesture to avoid
/// iOS "System gesture gate timed out" errors with long durations.
struct LongPressButton: View {
    let systemImage: String
    let duration: Double
    let brightness: Float
    let iconSize: CGFloat
    let action: () -> Void
    @Binding var holdProgress: CGFloat  // exposed to parent for large overlay

    @State private var isPressing = false
    @State private var fired = false  // true after action fires; blocks re-trigger until finger lifts
    @State private var holdTask: Task<Void, Never>?

    var body: some View {
        let frameSize = iconSize * 2.2
        Image(systemName: systemImage)
            .font(.system(size: iconSize, weight: .semibold))
            .foregroundColor(overlayTextColor(brightness: brightness))
            .frame(width: frameSize, height: frameSize)
            .overlay(
                Circle()
                    .trim(from: 0, to: holdProgress)
                    .stroke(Color.orange, style: StrokeStyle(lineWidth: 2.5, lineCap: .round))
                    .rotationEffect(.degrees(-90))
                    .padding(1)
            )
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { _ in
                        guard !isPressing, !fired else { return }
                        isPressing = true
                        withAnimation(.linear(duration: duration)) {
                            holdProgress = 1.0
                        }
                        holdTask = Task { @MainActor in
                            try? await Task.sleep(nanoseconds: UInt64(duration * 1_000_000_000))
                            guard !Task.isCancelled else { return }
                            action()
                            fired = true
                            isPressing = false
                            withAnimation(.easeOut(duration: 0.2)) {
                                holdProgress = 0
                            }
                        }
                    }
                    .onEnded { _ in
                        holdTask?.cancel()
                        holdTask = nil
                        isPressing = false
                        fired = false
                        withAnimation(.easeOut(duration: 0.2)) {
                            holdProgress = 0
                        }
                    }
            )
    }
}

// MARK: - Reset Countdown Overlay

/// Large centered countdown ring shown while holding a long-press button.
/// Displayed above the finger so the user can see progress clearly.
@ViewBuilder
func countdownOverlay(progress: CGFloat, systemImage: String, color: Color, iconSize: CGFloat, ringWidth: CGFloat) -> some View {
    let size = iconSize * 3
    ZStack {
        // Dimmed background (only visible while pressing)
        Circle()
            .fill(Color.black.opacity(progress > 0 ? 0.4 : 0))
            .frame(width: size, height: size)

        // Countdown ring (fills clockwise along the rim)
        Circle()
            .trim(from: 0, to: progress)
            .stroke(color, style: StrokeStyle(lineWidth: ringWidth, lineCap: .round))
            .rotationEffect(.degrees(-90))
            .frame(width: size - ringWidth, height: size - ringWidth)

        // Icon (only visible while pressing)
        Image(systemName: systemImage)
            .font(.system(size: iconSize, weight: .bold))
            .foregroundColor(color)
            .opacity(progress > 0 ? 1 : 0)
    }
    .allowsHitTesting(false)
}

// MARK: - Shared Rate Color

func rateColor(_ rate: Float) -> Color {
    if rate >= 40 {
        return .green
    } else if rate >= 20 {
        return .yellow
    } else if rate >= 5 {
        return .orange
    } else {
        return .red
    }
}

struct SimulationCellView: View {
    var instance: SimulationInstance
    var onTapFullscreen: () -> Void
    var onLoadModel: () -> Void

    @State private var resetProgress: CGFloat = 0
    @State private var stopProgress: CGFloat = 0

    #if os(tvOS)
    @FocusState private var isFocused: Bool
    #endif

    var body: some View {
        #if os(tvOS)
        // tvOS: Use Button for focus-based navigation
        Button(action: {
            if instance.isActive {
                onTapFullscreen()
            } else {
                onLoadModel()
            }
        }) {
            cellContent
        }
        .buttonStyle(TVCellButtonStyle())
        .focused($isFocused)
        #else
        // iOS/macOS: Use ZStack with gestures
        cellContent
        #endif
    }

    private var cellContent: some View {
        ZStack {
            if instance.isActive {
                // Active simulation view
                activeView
            } else {
                // Empty cell placeholder
                emptyView
            }
        }
        .clipShape(RoundedRectangle(cornerRadius: 12))
        .overlay(
            RoundedRectangle(cornerRadius: 12)
                .stroke(Color.gray.opacity(0.3), lineWidth: 1)
        )
    }

    // MARK: - Active View

    private var brightness: Float { instance.sceneBrightness }

    private var activeView: some View {
        ZStack {
            // Metal rendering view — hidden when blinded to save GPU
            if instance.isBlinded {
                Color.black
                Image(systemName: "eye.slash.fill")
                    .font(.system(size: 24))
                    .foregroundColor(.gray.opacity(0.3))
            } else {
                MuJoCoMetalView(dataSource: instance)
                    .allowsHitTesting(!instance.isLocked)
            }

            // Large centered countdown overlays (always present so trim animates)
            countdownOverlay(progress: resetProgress, systemImage: "arrow.counterclockwise", color: .orange, iconSize: 28, ringWidth: 4)
            countdownOverlay(progress: stopProgress, systemImage: "stop.fill", color: .red, iconSize: 28, ringWidth: 4)

            // Left control bar
            VStack {
                HStack {
                    VStack(spacing: 6) {
                        // Lock button (standalone toggle)
                        Button(action: { instance.isLocked.toggle() }) {
                            let frameSize: CGFloat = 10 * 2.2
                            Image(systemName: instance.isLocked ? "lock.fill" : "lock.open")
                                .font(.system(size: 10, weight: .semibold))
                                .foregroundColor(overlayTextColor(brightness: brightness))
                                .frame(width: frameSize, height: frameSize)
                        }
                        .buttonStyle(.plain)

                        // Eye button (toggle rendering)
                        Button(action: { instance.isBlinded.toggle() }) {
                            let frameSize: CGFloat = 10 * 2.2
                            Image(systemName: instance.isBlinded ? "eye.slash.fill" : "eye.fill")
                                .font(.system(size: 10, weight: .semibold))
                                .foregroundColor(overlayTextColor(brightness: brightness))
                                .frame(width: frameSize, height: frameSize)
                        }
                        .buttonStyle(.plain)

                        // Controls pill (visible when unlocked)
                        if !instance.isLocked {
                            VStack(spacing: 4) {
                                Button(action: { instance.togglePlayPause() }) {
                                    let isRunning = instance.state == .running
                                    let frameSize: CGFloat = 10 * 2.2
                                    Image(systemName: isRunning ? "pause.fill" : "play.fill")
                                        .font(.system(size: 10, weight: .semibold))
                                        .foregroundColor(overlayTextColor(brightness: brightness))
                                        .frame(width: frameSize, height: frameSize)
                                }
                                .buttonStyle(.plain)
                                LongPressButton(
                                    systemImage: "arrow.counterclockwise",
                                    duration: 3.0,
                                    brightness: brightness,
                                    iconSize: 10,
                                    action: { instance.reset() },
                                    holdProgress: $resetProgress
                                )
                                LongPressButton(
                                    systemImage: "stop.fill",
                                    duration: 3.0,
                                    brightness: brightness,
                                    iconSize: 10,
                                    action: { instance.unload() },
                                    holdProgress: $stopProgress
                                )
                                if !instance.isBlinded {
                                    Button(action: { instance.resetCamera() }) {
                                        let frameSize: CGFloat = 10 * 2.2
                                        Image(systemName: "camera.metering.center.weighted")
                                            .font(.system(size: 10, weight: .semibold))
                                            .foregroundColor(overlayTextColor(brightness: brightness))
                                            .frame(width: frameSize, height: frameSize)
                                    }
                                    .buttonStyle(.plain)
                                }
                            }
                            .padding(3)
                            .background(
                                RoundedRectangle(cornerRadius: 8)
                                    .fill(Color.black.opacity(0.35))
                            )
                        }
                    }
                    Spacer()
                }
                .padding(.leading, 6)
                .padding(.top, 32)
                Spacer()
            }

            // Info overlay — text color adapts to scene brightness
            VStack(spacing: 0) {
                // Top: title row, then metrics below (right-aligned)
                VStack(alignment: .leading, spacing: 4) {
                    // Title (full width, no wrap) — triple-click to enter fullscreen
                    Text(instance.modelName)
                        .font(.caption)
                        .fontWeight(.semibold)
                        .foregroundColor(overlayTextColor(brightness: brightness))
                        .lineLimit(1)

                    // Metrics (right-aligned, under title)
                    HStack {
                        Spacer()
                        performanceMetricsView
                    }
                }
                .padding(8)
                .background(
                    LinearGradient(
                        colors: [Color.black.opacity(0.6), Color.clear],
                        startPoint: .top,
                        endPoint: .bottom
                    )
                )

                Spacer()

                // Bottom-right: port + status + time
                HStack {
                    Spacer()
                    VStack(alignment: .trailing, spacing: 2) {
                        Text(verbatim: "Port :\(instance.port)")
                            .font(.system(size: 9, weight: .medium))
                            .foregroundColor(overlaySecondaryTextColor(brightness: brightness))
                        HStack(spacing: 4) {
                            Circle()
                                .fill(instance.state == .running ? Color.green : Color.yellow)
                                .frame(width: 6, height: 6)
                            Text(instance.stateDescription)
                                .font(.system(size: 9))
                                .foregroundColor(overlaySecondaryTextColor(brightness: brightness))
                        }
                        Text(formatSimulationTime(instance.simulationTime))
                            .font(.system(size: 8, weight: .medium, design: .monospaced))
                            .foregroundColor(overlayTextColor(brightness: brightness).opacity(0.8))
                    }
                    .padding(6)
                }
            }
        }
        .contentShape(Rectangle())
        .onTripleTap(dotColor: overlayTextColor(brightness: brightness), targetLabel: "fullscreen") {
            onTapFullscreen()
        }
    }

    // MARK: - Performance Metrics View

    private static let metricValueFont = Font.system(size: 8, weight: .medium, design: .monospaced)
    private static let metricLabelFont = Font.system(size: 8, weight: .medium)

    private var metricLabelColor: Color {
        overlayTertiaryTextColor(brightness: brightness)
    }

    private var performanceMetricsView: some View {
        Grid(horizontalSpacing: 3, verticalSpacing: 2) {
            GridRow {
                Text(String(format: "%7.1f", instance.stepsPerSecondFloat))
                    .font(Self.metricValueFont)
                    .foregroundColor(stepsPerSecondColor)
                    .gridColumnAlignment(.trailing)
                Text("fps")
                    .font(Self.metricLabelFont)
                    .foregroundColor(metricLabelColor)
                    .gridColumnAlignment(.leading)
                Text("SIM")
                    .font(Self.metricLabelFont)
                    .foregroundColor(metricLabelColor)
                    .gridColumnAlignment(.leading)
            }
            if instance.hasClient || instance.txRate > 0 {
                GridRow {
                    Text(String(format: "%7.1f", instance.txRate))
                        .font(Self.metricValueFont)
                        .foregroundColor(rateColor(instance.txRate))
                    Text("fps")
                        .font(Self.metricLabelFont)
                        .foregroundColor(metricLabelColor)
                    Text("State TX")
                        .font(Self.metricLabelFont)
                        .foregroundColor(metricLabelColor)
                }
            }
            if instance.hasClient || instance.rxRate > 0 {
                GridRow {
                    Text(String(format: "%7.1f", instance.rxRate))
                        .font(Self.metricValueFont)
                        .foregroundColor(rateColor(instance.rxRate))
                    Text("fps")
                        .font(Self.metricLabelFont)
                        .foregroundColor(metricLabelColor)
                    Text("Control RX")
                        .font(Self.metricLabelFont)
                        .foregroundColor(metricLabelColor)
                }
            }
        }
        .fixedSize()
    }

    private var stepsPerSecondColor: Color {
        let sps = instance.stepsPerSecondFloat
        if sps >= 400 {
            return .green       // Excellent
        } else if sps >= 200 {
            return .yellow      // Good
        } else if sps >= 100 {
            return .orange      // Okay
        } else {
            return .red         // Slow
        }
    }

    // MARK: - Empty View

    private var emptyView: some View {
        VStack(spacing: 16) {
            Image(systemName: "cube.transparent")
                .font(.system(size: 40))
                .foregroundColor(.gray.opacity(0.5))

            #if os(tvOS)
            Text("Press to Load Model")
                .font(.caption)
                .foregroundColor(.gray)
            #else
            Text("Empty")
                .font(.caption)
                .foregroundColor(.gray)

            Button(action: onLoadModel) {
                Label("Load Model", systemImage: "plus.circle.fill")
                    .font(.caption)
            }
            .buttonStyle(.bordered)
            #endif
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(Color.black.opacity(0.9))
        .contentShape(Rectangle())
        .onTripleTap(dotColor: .gray, targetLabel: "fullscreen") {
            onTapFullscreen()
        }
    }
}

// MARK: - tvOS Button Style

#if os(tvOS)
struct TVCellButtonStyle: ButtonStyle {
    @Environment(\.isFocused) var isFocused

    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .scaleEffect(isFocused ? 1.05 : 1.0)
            .shadow(color: isFocused ? .white.opacity(0.5) : .clear, radius: 10)
            .animation(.easeInOut(duration: 0.2), value: isFocused)
            .overlay(
                RoundedRectangle(cornerRadius: 12)
                    .stroke(isFocused ? Color.white : Color.clear, lineWidth: 4)
            )
    }
}
#endif

// MARK: - Preview

#if DEBUG
#Preview {
    SimulationCellView(
        instance: SimulationInstance(id: 0),
        onTapFullscreen: {},
        onLoadModel: {}
    )
    .frame(width: 300, height: 200)
}
#endif
