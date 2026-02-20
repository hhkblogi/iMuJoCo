// performance_stats_bar.swift
// Compact performance stats overlay for the simulation grid

import Metal
import SwiftUI

struct PerformanceStatsBar: View {
    let instances: [SimulationInstance]
    @State private var memoryMB: Double = 0
    @State private var cpuUsage: Double = 0
    @State private var gpuMemoryMB: Double = 0

    private var activeInstances: [SimulationInstance] {
        instances.filter { $0.state == .running }
    }

    private var avgRenderFPS: Double {
        let active = activeInstances
        guard !active.isEmpty else { return 0 }
        return active.reduce(0.0) { $0 + $1.renderFPS } / Double(active.count)
    }

    var body: some View {
        HStack(spacing: 12) {
            statCapsule(
                value: String(format: "%.0f", avgRenderFPS),
                unit: "fps",
                label: "SCR",
                color: fpsColor(avgRenderFPS)
            )

            statCapsule(
                value: String(format: "%.0f", cpuUsage),
                unit: "%",
                label: "CPU",
                color: cpuColor(cpuUsage)
            )

            memCapsule
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 6)
        .background(Color.black.opacity(0.6))
        .task {
            let device = MTLCreateSystemDefaultDevice()
            while !Task.isCancelled {
                let mem = processMemoryMB()
                let cpu = processCPUUsage()
                let gpuMem = Double(device?.currentAllocatedSize ?? 0) / (1024 * 1024)
                await MainActor.run {
                    memoryMB = mem
                    cpuUsage = cpu
                    gpuMemoryMB = gpuMem
                }
                try? await Task.sleep(nanoseconds: 500_000_000)
            }
        }
    }

    // MARK: - Stat Capsule

    private func statCapsule(value: String, unit: String, label: String, color: Color) -> some View {
        HStack(spacing: 3) {
            Text(label)
                .font(.system(size: 9, weight: .medium))
                .foregroundColor(.white.opacity(0.5))
            Text(value)
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundColor(color)
            if !unit.isEmpty {
                Text(unit)
                    .font(.system(size: 8, weight: .medium))
                    .foregroundColor(.white.opacity(0.4))
            }
        }
    }

    // MARK: - Memory Capsule

    private var memCapsule: some View {
        HStack(spacing: 3) {
            Text("MEM")
                .font(.system(size: 9, weight: .medium))
                .foregroundColor(.white.opacity(0.5))
            Text(String(format: "%.0f", memoryMB))
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundColor(memColor(memoryMB))
            Text("MB")
                .font(.system(size: 8, weight: .medium))
                .foregroundColor(.white.opacity(0.4))
            // GMEM sub-group with tighter spacing
            HStack(spacing: 2) {
                Text("(GMEM")
                    .font(.system(size: 8, weight: .medium))
                    .foregroundColor(.white.opacity(0.4))
                Text(String(format: "%.0f", gpuMemoryMB))
                    .font(.system(size: 9, weight: .medium, design: .monospaced))
                    .foregroundColor(gpuMemColor(gpuMemoryMB))
                Text("MB)")
                    .font(.system(size: 8, weight: .medium))
                    .foregroundColor(.white.opacity(0.4))
            }
        }
    }

    // MARK: - Color Thresholds

    private func fpsColor(_ fps: Double) -> Color {
        if fps >= 55 { return .green }
        else if fps >= 30 { return .yellow }
        else if fps >= 15 { return .orange }
        else { return .red }
    }

    private func cpuColor(_ pct: Double) -> Color {
        if pct < 100 { return .green }
        else if pct < 200 { return .yellow }
        else if pct < 400 { return .orange }
        else { return .red }
    }

    private func memColor(_ mb: Double) -> Color {
        if mb < 1024 { return .green }
        else if mb < 1536 { return .yellow }
        else if mb < 2048 { return .orange }
        else { return .red }
    }

    private func gpuMemColor(_ mb: Double) -> Color {
        if mb < 1024 { return .green }
        else if mb < 1536 { return .yellow }
        else if mb < 2048 { return .orange }
        else { return .red }
    }
}
