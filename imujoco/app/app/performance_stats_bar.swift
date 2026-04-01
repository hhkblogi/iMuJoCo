// performance_stats_bar.swift
// Compact performance stats overlay for the simulation grid

import Metal
import SwiftUI
import core

struct PerformanceStatsBar: View {
    let instances: [SimulationInstance]
    var grpcRpcCount: () -> UInt64
    var grpcPort: () -> UInt16
    @State private var memoryMB: Double = 0
    @State private var cpuUsage: Double = 0
    @State private var gpuMemoryMB: Double = 0
    @State private var lastGrpcRpcCount: UInt64 = 0
    @State private var grpcActive: Bool = false
    @State private var lastSyncResponsesSent: UInt64 = 0
    @State private var syncActive: Bool = false
    @State private var syncPort: UInt16 = 0
    @State private var syncRunning: Bool = false
    @State private var syncStats: MJSyncStats?
    @State private var syncRollingIndex: Int = 0
    @State private var syncRollingTick: Int = 0

    private static let metalDevice = MTLCreateSystemDefaultDevice()

    private var activeInstances: [SimulationInstance] {
        instances.filter { $0.state == .running }
    }

    private var avgRenderFPS: Double {
        let active = activeInstances
        guard !active.isEmpty else { return 0 }
        return active.reduce(0.0) { $0 + $1.renderFPS } / Double(active.count)
    }

    var body: some View {
        VStack(spacing: 0) {
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

                Spacer()

                statCapsule(
                    value: ":\(grpcPort())",
                    unit: "",
                    label: "gRPC",
                    color: grpcActive ? .green : .white
                )
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 2)

            syncStatusLine
                .padding(.horizontal, 16)
                .padding(.bottom, 2)
        }
        .background(Color.black.opacity(0.6))
        .task {
            while !Task.isCancelled {
                let mem = processMemoryMB()
                let cpu = processCPUUsage()
                let gpuMem = Double(Self.metalDevice?.currentAllocatedSize ?? 0) / (1024 * 1024)
                let currentRpc = grpcRpcCount()
                await MainActor.run {
                    memoryMB = mem
                    cpuUsage = cpu
                    gpuMemoryMB = gpuMem
                    grpcActive = currentRpc != lastGrpcRpcCount
                    lastGrpcRpcCount = currentRpc

                    // Rotate through active sync instances every 5s (10 ticks).
                    // All @State reads/writes are on MainActor to avoid data races.
                    // Snapshot syncStats once per instance into tuples to avoid
                    // repeated Swift-C++ interop calls within this tick.
                    let activeSyncInstances: [(instance: SimulationInstance, stats: MJSyncStats)] = instances.compactMap { inst in
                        guard inst.state == .running, let stats = inst.runtime?.syncStats, stats.isRunning else { return nil }
                        return (inst, stats)
                    }
                    let tick = syncRollingTick + 1
                    let rollingIdx: Int
                    if activeSyncInstances.isEmpty {
                        rollingIdx = 0
                    } else {
                        rollingIdx = (tick % 10 == 0)
                            ? (syncRollingIndex + 1) % activeSyncInstances.count
                            : syncRollingIndex % max(activeSyncInstances.count, 1)
                    }
                    let syncStats: MJSyncStats? = activeSyncInstances.isEmpty
                        ? nil
                        : activeSyncInstances[rollingIdx].stats
                    syncRunning = syncStats?.isRunning ?? false
                    let newPort = syncStats?.port ?? 0
                    if newPort != syncPort {
                        lastSyncResponsesSent = syncStats?.responsesSent ?? 0
                    }
                    syncPort = newPort
                    syncActive = syncStats != nil && syncStats!.responsesSent != lastSyncResponsesSent
                    lastSyncResponsesSent = syncStats?.responsesSent ?? 0
                    self.syncStats = syncStats
                    syncRollingTick = tick
                    syncRollingIndex = rollingIdx
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

    // MARK: - Sync Status Line

    private var syncStatusLine: some View {
        // Lock Failed: servo has had enough exchanges to converge but hasn't locked.
        // 30 exchanges = full lock window (3s at 10Hz) — if still not locked, something is wrong.
        let lockFailed: Bool = {
            guard syncActive, let s = syncStats else { return false }
            return !s.driverLocked && s.driverExchanges > 30
        }()

        let color: Color = {
            if !syncRunning { return .gray }
            if lockFailed { return .red }
            if syncActive, let s = syncStats, s.driverLocked { return .green }
            if syncActive { return .white }
            return .white.opacity(0.4)
        }()

        let text: String = {
            guard syncRunning else { return "PTP off" }
            guard let s = syncStats else { return "PTP :\(syncPort)" }

            // Fixed-width fields to prevent bar width from jumping.
            // syncActive = responses still being sent; driverLocked = last feedback said locked.
            let status: String
            if !syncActive {
                status = "Idle"
            } else if s.driverLocked {
                status = "Locked"
            } else if lockFailed {
                status = "Lock Failed"
            } else {
                status = "Syncing"
            }

            if syncActive && s.driverExchanges > 0 {
                let delayMs = Double(s.driverDelayUs) / 1000.0
                return String(
                    format: "PTP :%u  %@  Delay: %5.1f ms  Rate: %+6d ppm  Jitter: %5.1f us",
                    syncPort, status, delayMs, s.serverRateRatioPpm, s.driverJitterUs
                )
            } else {
                return "PTP :\(syncPort)  \(status)"
            }
        }()

        return Text(text)
            .font(.system(size: 9, weight: .medium, design: .monospaced))
            .foregroundColor(color)
            .frame(maxWidth: .infinity, alignment: .leading)
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
