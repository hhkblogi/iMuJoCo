import SwiftUI
import core
#if canImport(UIKit)
import UIKit
#endif
#if canImport(BackgroundTasks)
import BackgroundTasks
#endif

// MARK: - Focused Value (macOS menu bar → grid manager)

#if os(macOS)
struct FocusedGridManagerKey: FocusedValueKey {
    typealias Value = SimulationGridManager
}

extension FocusedValues {
    var gridManager: SimulationGridManager? {
        get { self[FocusedGridManagerKey.self] }
        set { self[FocusedGridManagerKey.self] = newValue }
    }
}
#endif

// MARK: - App

@main
struct MuJoCoApp: App {
    @State private var gridManager = SimulationGridManager()
    @Environment(\.scenePhase) private var scenePhase
    #if os(iOS)
    @AppStorage("caffeineMode") private var caffeineMode: Int = 1  // 0=off, 1=half, 2=full
    #endif

    init() {
        // Register UserDefaults so .integer(forKey:) returns correct values
        // even before @AppStorage writes anything (fresh install).
        UserDefaults.standard.register(defaults: [
            "videoTransport": 3,   // Off
            "defaultLocked": false
        ])
        #if os(iOS)
        SimulationGridManager.registerBackgroundTask()
        #endif

        // Sync server is now per-instance — started when runtime is created
    }

    var body: some Scene {
        WindowGroup {
            ContentView(gridManager: gridManager)
                #if os(macOS)
                .focusedValue(\.gridManager, gridManager)
                #endif
                .onChange(of: scenePhase) { _, newPhase in
                    switch newPhase {
                    case .background:
                        // Always suspend GPU-based video capture — Metal command
                        // buffers are rejected from background regardless of
                        // caffeine mode (physics can continue without GPU).
                        gridManager.suspendVideoCapture()
                        #if os(iOS)
                        if caffeineMode >= 2 {
                            gridManager.beginCaffeineBackground()
                        } else {
                            gridManager.pauseAll()
                        }
                        #else
                        gridManager.beginBackgroundExecution()
                        #endif
                    case .active:
                        #if os(iOS)
                        if caffeineMode >= 2 {
                            gridManager.endCaffeineBackground()
                        } else {
                            gridManager.resumeAll()
                        }
                        #else
                        gridManager.endBackgroundExecution()
                        #endif
                        // Resume GPU-based video capture now that we're foreground
                        gridManager.resumeVideoCapture()
                    case .inactive:
                        break
                    @unknown default:
                        break
                    }
                }
                #if os(iOS)
                .onReceive(NotificationCenter.default.publisher(for: .caffeineBackgroundTaskLaunched)) { notification in
                    if let task = notification.object as? BGProcessingTask {
                        gridManager.handleCaffeineTask(task)
                    }
                }
                .onChange(of: caffeineMode) { _, level in
                    UIApplication.shared.isIdleTimerDisabled = level >= 1
                    if level < 2 {
                        gridManager.endCaffeineBackground()
                    }
                }
                .onAppear {
                    UIApplication.shared.isIdleTimerDisabled = caffeineMode >= 1
                }
                #endif
        }
        #if os(macOS)
        .defaultSize(width: 900, height: 700)
        .commands {
            SimulationCommands()
            ViewCommands()
            CameraCommands()
        }
        #endif

        #if os(macOS)
        Settings {
            MacSettingsWrapper()
        }
        #endif
    }
}

// MARK: - macOS Menu Commands

#if os(macOS)

struct MacSettingsWrapper: View {
    @AppStorage("defaultView") private var defaultView: Int = 0
    @AppStorage("defaultLocked") private var defaultLocked: Bool = true
    @FocusedValue(\.gridManager) private var gridManager

    var body: some View {
        SettingsView(
            defaultView: $defaultView,
            defaultLocked: $defaultLocked,
            onDismiss: {},
            gridManager: gridManager
        )
    }
}

struct SimulationCommands: Commands {
    @FocusedValue(\.gridManager) private var gridManager

    private var activeInstance: SimulationInstance? {
        gridManager?.fullscreenInstance
    }

    var body: some Commands {
        CommandMenu("Simulation") {
            Button("Play/Pause All") {
                guard let gm = gridManager else { return }
                let hasRunning = gm.instances.contains { $0.state == .running }
                if hasRunning {
                    gm.pauseAll()
                } else {
                    gm.resumeAll()
                }
            }
            .keyboardShortcut("p", modifiers: .command)

            Divider()

            Button("Play/Pause") {
                activeInstance?.togglePlayPause()
            }
            .keyboardShortcut(.space, modifiers: [])
            .disabled(activeInstance == nil)

            Button("Step") {
                activeInstance?.step()
            }
            .keyboardShortcut(".", modifiers: .command)
            .disabled(activeInstance == nil)

            Button("Reset Simulation") {
                activeInstance?.reset()
            }
            .keyboardShortcut("r", modifiers: [.command, .shift])
            .disabled(activeInstance == nil)

            Button("Unload") {
                activeInstance?.unload()
            }
            .keyboardShortcut(.delete, modifiers: .command)
            .disabled(activeInstance == nil)
        }
    }
}

struct ViewCommands: Commands {
    @FocusedValue(\.gridManager) private var gridManager
    @AppStorage("showStatsBar") private var showStatsBar: Bool = true

    var body: some Commands {
        CommandMenu("View") {
            Toggle("Stats Bar", isOn: $showStatsBar)
                .keyboardShortcut("s", modifiers: [.command, .shift])

            Divider()

            Button("Grid View") {
                gridManager?.exitFullscreen()
            }
            .keyboardShortcut("1", modifiers: .command)

            ForEach(0..<4) { index in
                Button("Fullscreen \(index + 1)") {
                    gridManager?.enterFullscreen(index: index)
                }
                .keyboardShortcut(KeyEquivalent(Character("\(index + 2)")), modifiers: .command)
            }
        }
    }
}

struct CameraCommands: Commands {
    @FocusedValue(\.gridManager) private var gridManager

    private var activeInstance: SimulationInstance? {
        gridManager?.fullscreenInstance
    }

    var body: some Commands {
        CommandMenu("Camera") {
            Button("Reset Camera") {
                activeInstance?.resetCamera()
            }
            .keyboardShortcut("r", modifiers: .command)
            .disabled(activeInstance == nil)

            Divider()

            Button(activeInstance?.isLocked == true ? "Unlock Camera" : "Lock Camera") {
                activeInstance?.isLocked.toggle()
            }
            .keyboardShortcut("l", modifiers: .command)
            .disabled(activeInstance == nil)

            Button(activeInstance?.isBlinded == true ? "Show Render" : "Hide Render") {
                activeInstance?.isBlinded.toggle()
            }
            .keyboardShortcut("b", modifiers: .command)
            .disabled(activeInstance == nil)
        }
    }
}

#endif
