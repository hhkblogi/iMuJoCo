// content_view.swift
// Main content view switching between grid and fullscreen

import SwiftUI

struct ContentView: View {
    @State private var gridManager = SimulationGridManager()

    var body: some View {
        ZStack {
            // Grid view (always present, but hidden when fullscreen)
            SimulationGridView(gridManager: gridManager)
                .opacity(gridManager.isFullscreen ? 0 : 1)

            // Fullscreen view
            if let instance = gridManager.fullscreenInstance {
                FullscreenSimulationView(
                    instance: instance,
                    onExit: {
                        withAnimation(.easeInOut(duration: 0.3)) {
                            gridManager.exitFullscreen()
                        }
                    }
                )
                .transition(.opacity)
            }
        }
        .background(Color.black)
        .preferredColorScheme(.dark)
    }
}

// MARK: - Preview

#if DEBUG
#Preview {
    ContentView()
}
#endif
