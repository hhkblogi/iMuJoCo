import SwiftUI
#if canImport(UIKit)
import UIKit
#endif

@main
struct MuJoCoApp: App {
    @State private var gridManager = SimulationGridManager()
    @Environment(\.scenePhase) private var scenePhase
    @AppStorage("caffeineMode") private var caffeineMode: Int = 1  // 0=off, 1=half, 2=full

    var body: some Scene {
        WindowGroup {
            ContentView(gridManager: gridManager)
                .onChange(of: scenePhase) { _, newPhase in
                    switch newPhase {
                    case .background:
                        #if os(iOS)
                        if caffeineMode >= 2 {
                            gridManager.beginCaffeineBackground()
                        } else {
                            gridManager.beginBackgroundExecution()
                        }
                        #else
                        gridManager.beginBackgroundExecution()
                        #endif
                    case .active:
                        gridManager.endBackgroundExecution()
                        #if os(iOS)
                        gridManager.endCaffeineBackground()
                        #endif
                    case .inactive:
                        break
                    @unknown default:
                        break
                    }
                }
                #if os(iOS)
                .onChange(of: caffeineMode) { _, level in
                    UIApplication.shared.isIdleTimerDisabled = level >= 1
                }
                .onAppear {
                    UIApplication.shared.isIdleTimerDisabled = caffeineMode >= 1
                }
                #endif
        }
    }
}
