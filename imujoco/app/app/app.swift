import SwiftUI

@main
struct MuJoCoApp: App {
    @State private var gridManager = SimulationGridManager()
    @Environment(\.scenePhase) private var scenePhase
    @AppStorage("caffeineMode") private var caffeineMode: Bool = false

    var body: some Scene {
        WindowGroup {
            ContentView(gridManager: gridManager)
                .onChange(of: scenePhase) { _, newPhase in
                    switch newPhase {
                    case .background:
                        #if os(iOS)
                        if caffeineMode {
                            gridManager.beginCaffeineBackground()
                        } else {
                            gridManager.beginBackgroundExecution()
                        }
                        #else
                        gridManager.beginBackgroundExecution()
                        #endif
                    case .inactive, .active:
                        gridManager.endBackgroundExecution()
                        #if os(iOS)
                        gridManager.endCaffeineBackground()
                        #endif
                    @unknown default:
                        break
                    }
                }
                #if os(iOS)
                .onChange(of: caffeineMode) { _, enabled in
                    UIApplication.shared.isIdleTimerDisabled = enabled
                }
                .onAppear {
                    UIApplication.shared.isIdleTimerDisabled = caffeineMode
                }
                #endif
        }
    }
}
