import SwiftUI

@main
struct MuJoCoApp: App {
    @State private var gridManager = SimulationGridManager()
    @Environment(\.scenePhase) private var scenePhase

    var body: some Scene {
        WindowGroup {
            ContentView(gridManager: gridManager)
                .onChange(of: scenePhase) { _, newPhase in
                    switch newPhase {
                    case .background:
                        gridManager.beginBackgroundExecution()
                    case .active:
                        gridManager.endBackgroundExecution()
                    default:
                        break
                    }
                }
        }
    }
}
