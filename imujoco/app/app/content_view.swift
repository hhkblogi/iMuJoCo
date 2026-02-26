// content_view.swift
// Main content view switching between grid and fullscreen

import SwiftUI

struct ContentView: View {
    var gridManager: SimulationGridManager
    @State private var showingFullscreenModelPicker = false
    @State private var fullscreenErrorMessage = ""
    @State private var showingFullscreenError = false
    @State private var popoverWarmup = false
    @AppStorage("defaultView") private var defaultView: Int = 0

    var body: some View {
        ZStack {
            // Grid view (removed from hierarchy when fullscreen to save GPU)
            if !gridManager.isFullscreen {
                SimulationGridView(gridManager: gridManager)
            }

            // Fullscreen view
            if let fsId = gridManager.fullscreenInstanceId,
               let instance = gridManager.instance(at: fsId) {
                FullscreenSimulationView(
                    instance: instance,
                    instanceIndex: fsId,
                    onExit: {
                        withAnimation(.easeInOut(duration: 0.3)) {
                            gridManager.exitFullscreen()
                        }
                    },
                    onSwitchInstance: { index in
                        gridManager.enterFullscreen(index: index)
                    },
                    onLoadModel: {
                        showingFullscreenModelPicker = true
                    }
                )
                .transition(.opacity)
            }
        }
        .background(Color.black)
        .preferredColorScheme(.dark)
        #if !os(tvOS)
        // Hidden popover warmup — pre-initializes UIKit's presentation
        // controller to avoid "System gesture gate timed out" on first tap.
        .popover(isPresented: $popoverWarmup) {
            Color.clear.frame(width: 1, height: 1)
        }
        .sheet(isPresented: $showingFullscreenModelPicker) {
            ModelPickerView(
                modelGroups: gridManager.bundledModelsBySource,
                onSelectModel: { modelName in
                    loadFullscreenModel(name: modelName)
                },
                onDismiss: {
                    showingFullscreenModelPicker = false
                }
            )
        }
        #endif
        .onAppear {
            if defaultView >= 1, defaultView <= 4 {
                gridManager.enterFullscreen(index: defaultView - 1)
            }
            // Warm up UIKit popover presentation infrastructure at app launch
            DispatchQueue.main.async {
                popoverWarmup = true
                DispatchQueue.main.async {
                    popoverWarmup = false
                }
            }
        }
        .alert("Failed to Load Model", isPresented: $showingFullscreenError) {
            Button("OK", role: .cancel) {}
        } message: {
            Text(fullscreenErrorMessage)
        }
    }

    private func loadFullscreenModel(name: String) {
        showingFullscreenModelPicker = false
        guard let index = gridManager.fullscreenInstanceId else { return }
        Task {
            do {
                try await gridManager.loadBundledModel(at: index, name: name)
            } catch {
                await MainActor.run {
                    fullscreenErrorMessage = error.localizedDescription
                    showingFullscreenError = true
                }
            }
        }
    }
}

// MARK: - Preview

#if DEBUG
#Preview {
    ContentView(gridManager: SimulationGridManager())
}
#endif
