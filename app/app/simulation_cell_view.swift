// simulation_cell_view.swift
// Individual cell view for simulation grid

import SwiftUI
import render

struct SimulationCellView: View {
    var instance: SimulationInstance
    var onTapFullscreen: () -> Void
    var onLoadModel: () -> Void

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

    private var activeView: some View {
        ZStack {
            // Metal rendering view
            MuJoCoMetalView(dataSource: instance)

            // Info overlay
            VStack {
                // Top bar with model name and port
                HStack {
                    Text(instance.modelName)
                        .font(.caption)
                        .fontWeight(.semibold)
                        .foregroundColor(.white)

                    Spacer()

                    Text(":\(instance.port)")
                        .font(.caption2)
                        .fontWeight(.medium)
                        .foregroundColor(.white.opacity(0.7))
                        .padding(.horizontal, 6)
                        .padding(.vertical, 2)
                        .background(Color.black.opacity(0.5))
                        .clipShape(Capsule())
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

                // Performance metrics overlay (top-right corner)
                HStack {
                    Spacer()
                    performanceMetricsView
                }
                .padding(.horizontal, 8)

                Spacer()

                // Bottom bar with status and time
                HStack {
                    // Status indicator
                    Circle()
                        .fill(instance.state == .running ? Color.green : Color.yellow)
                        .frame(width: 8, height: 8)

                    Text(instance.stateDescription)
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.8))

                    Spacer()

                    Text(String(format: "%.2fs", instance.simulationTime))
                        .font(.caption2)
                        .monospacedDigit()
                        .foregroundColor(.white.opacity(0.8))
                }
                .padding(8)
                .background(
                    LinearGradient(
                        colors: [Color.clear, Color.black.opacity(0.6)],
                        startPoint: .top,
                        endPoint: .bottom
                    )
                )
            }
        }
        .contentShape(Rectangle())
        #if os(iOS)
        .onTapGesture(count: 2) {
            onTapFullscreen()
        }
        #endif
        #if os(macOS)
        .onTapGesture(count: 2) {
            onTapFullscreen()
        }
        #endif
    }

    // MARK: - Performance Metrics View

    private var performanceMetricsView: some View {
        VStack(alignment: .trailing, spacing: 2) {
            // Physics steps per second
            HStack(spacing: 4) {
                Text("SIM")
                    .font(.system(size: 8, weight: .medium))
                    .foregroundColor(.white.opacity(0.6))
                Text("\(instance.stepsPerSecond)")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundColor(stepsPerSecondColor)
                Text("Hz")
                    .font(.system(size: 8, weight: .medium))
                    .foregroundColor(.white.opacity(0.6))
            }

            // Model timestep (shows expected real-time rate)
            HStack(spacing: 4) {
                Text("dt")
                    .font(.system(size: 8, weight: .medium))
                    .foregroundColor(.white.opacity(0.6))
                Text(String(format: "%.1fms", instance.timestep * 1000))
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundColor(.white.opacity(0.8))
            }
        }
        .padding(.horizontal, 6)
        .padding(.vertical, 4)
        .background(Color.black.opacity(0.7))
        .clipShape(RoundedRectangle(cornerRadius: 4))
    }

    private var stepsPerSecondColor: Color {
        let sps = instance.stepsPerSecond
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
