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
            VStack(spacing: 0) {
                // Top bar
                HStack(alignment: .top) {
                    // Left: model name + status
                    VStack(alignment: .leading, spacing: 3) {
                        Text(instance.modelName)
                            .font(.caption)
                            .fontWeight(.semibold)
                            .foregroundColor(.white)

                        HStack(spacing: 4) {
                            Circle()
                                .fill(instance.state == .running ? Color.green : Color.yellow)
                                .frame(width: 6, height: 6)
                            Text(instance.stateDescription)
                                .font(.system(size: 9))
                                .foregroundColor(.white.opacity(0.7))
                        }
                    }

                    Spacer()

                    // Right: port badge + metrics
                    VStack(alignment: .trailing, spacing: 4) {
                        Text(verbatim: ":\(instance.port)")
                            .font(.caption2)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.7))
                            .padding(.horizontal, 6)
                            .padding(.vertical, 2)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Capsule())

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

                // Bottom bar: time only (right-aligned)
                HStack {
                    Spacer()

                    Text(formatSimulationTime(instance.simulationTime))
                        .font(.system(size: 10, weight: .medium, design: .monospaced))
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

    private static let metricValueFont = Font.system(size: 10, weight: .bold, design: .monospaced)
    private static let metricLabelFont = Font.system(size: 8, weight: .medium)
    private static let metricLabelColor = Color.white.opacity(0.5)

    private var performanceMetricsView: some View {
        Grid(horizontalSpacing: 3, verticalSpacing: 2) {
            GridRow {
                Text(String(format: "%.1f", instance.stepsPerSecondFloat))
                    .font(Self.metricValueFont)
                    .foregroundColor(stepsPerSecondColor)
                    .gridColumnAlignment(.trailing)
                Text("fps")
                    .font(Self.metricLabelFont)
                    .foregroundColor(Self.metricLabelColor)
                    .gridColumnAlignment(.leading)
                Text("SIM")
                    .font(Self.metricLabelFont)
                    .foregroundColor(Self.metricLabelColor)
                    .gridColumnAlignment(.leading)
            }
            if instance.hasClient || instance.txRate > 0 {
                GridRow {
                    Text(String(format: "%.1f", instance.txRate))
                        .font(Self.metricValueFont)
                        .foregroundColor(rateColor(instance.txRate))
                    Text("fps")
                        .font(Self.metricLabelFont)
                        .foregroundColor(Self.metricLabelColor)
                    Text("State TX")
                        .font(Self.metricLabelFont)
                        .foregroundColor(Self.metricLabelColor)
                }
            }
            if instance.hasClient || instance.rxRate > 0 {
                GridRow {
                    Text(String(format: "%.1f", instance.rxRate))
                        .font(Self.metricValueFont)
                        .foregroundColor(rateColor(instance.rxRate))
                    Text("fps")
                        .font(Self.metricLabelFont)
                        .foregroundColor(Self.metricLabelColor)
                    Text("Control RX")
                        .font(Self.metricLabelFont)
                        .foregroundColor(Self.metricLabelColor)
                }
            }
        }
        .fixedSize()
        .padding(.horizontal, 6)
        .padding(.vertical, 4)
        .background(Color.black.opacity(0.7))
        .clipShape(RoundedRectangle(cornerRadius: 4))
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
