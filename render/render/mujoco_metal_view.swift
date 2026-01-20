// mujoco_metal_view.swift
// SwiftUI wrapper for Metal-based MuJoCo rendering

import SwiftUI
import MetalKit
import MJCPhysicsRuntime

// MARK: - Runtime Protocol

/// Protocol for physics runtime that provides frame data for rendering.
///
/// Implement this protocol to connect your physics runtime to the Metal view.
/// The renderer will call `latestFrame` each frame to get geometry data for rendering.
/// If `latestFrame` returns nil, the renderer falls back to the legacy `scenePointer` API.
///
/// ## Camera Control
/// The protocol includes camera properties that gesture handlers will update.
/// Your implementation should apply these values to the underlying camera system.
///
/// ## Example Implementation
/// ```swift
/// class MyRuntime: MJRenderDataSource {
///     var latestFrame: UnsafePointer<MJFrameData>? {
///         return mjc_runtime_get_latest_frame(handle)
///     }
///     // ... implement other requirements
/// }
/// ```
public protocol MJRenderDataSource: AnyObject {
    /// Get the latest frame data for rendering (non-blocking).
    /// Returns nil if no frame is available yet.
    var latestFrame: UnsafePointer<MJFrameData>? { get }

    /// Legacy scene pointer for backwards compatibility.
    /// Used as fallback when `latestFrame` returns nil.
    var scenePointer: UnsafePointer<mjvScene>? { get }

    /// Legacy camera pointer for backwards compatibility.
    var cameraPointer: UnsafeMutablePointer<mjvCamera>? { get }

    /// Camera azimuth angle in degrees (horizontal rotation).
    var cameraAzimuth: Double { get set }

    /// Camera elevation angle in degrees (vertical rotation).
    var cameraElevation: Double { get set }

    /// Camera distance from the lookat point.
    /// Must remain positive; gesture handlers clamp to `minCameraDistance`.
    var cameraDistance: Double { get set }

    /// Reset camera to default position.
    func resetCamera()
}

/// Minimum camera distance to prevent rendering issues from zero/negative distances.
private let minCameraDistance: Double = 0.1

// MARK: - Metal View Representable

#if os(iOS) || os(tvOS)
/// SwiftUI wrapper for MuJoCo Metal rendering on iOS and tvOS.
///
/// Use this view in your SwiftUI hierarchy to display MuJoCo physics simulations.
/// The view automatically handles gestures for camera control:
/// - **iOS**: Pan to rotate, pinch to zoom, double-tap to reset
/// - **tvOS**: Swipe to rotate, vertical swipe to zoom, tap to reset
///
/// ## Usage
/// ```swift
/// struct ContentView: View {
///     @StateObject var runtime = MyMuJoCoRuntime()
///
///     var body: some View {
///         MuJoCoMetalView(dataSource: runtime)
///     }
/// }
/// ```
public struct MuJoCoMetalView: UIViewRepresentable {
    /// The data source providing frame data and camera control.
    public var dataSource: MJRenderDataSource

    /// Creates a new MuJoCo Metal view with the specified data source.
    public init(dataSource: MJRenderDataSource) {
        self.dataSource = dataSource
    }

    public func makeUIView(context: Context) -> MuJoCoMTKView {
        let view = MuJoCoMTKView()
        view.dataSource = dataSource
        return view
    }

    public func updateUIView(_ uiView: MuJoCoMTKView, context: Context) {
        uiView.dataSource = dataSource
    }
}
#endif

#if os(macOS)
/// SwiftUI wrapper for MuJoCo Metal rendering on macOS.
///
/// Use this view in your SwiftUI hierarchy to display MuJoCo physics simulations.
/// The view automatically handles mouse/keyboard for camera control:
/// - **Drag**: Rotate camera
/// - **Right-drag/Scroll**: Zoom
/// - **Press 'R'**: Reset camera
public struct MuJoCoMetalView: NSViewRepresentable {
    /// The data source providing frame data and camera control.
    public var dataSource: MJRenderDataSource

    /// Creates a new MuJoCo Metal view with the specified data source.
    public init(dataSource: MJRenderDataSource) {
        self.dataSource = dataSource
    }

    public func makeNSView(context: Context) -> MuJoCoMTKView {
        let view = MuJoCoMTKView()
        view.dataSource = dataSource
        return view
    }

    public func updateNSView(_ nsView: MuJoCoMTKView, context: Context) {
        nsView.dataSource = dataSource
    }
}
#endif

// MARK: - Custom MTKView

public class MuJoCoMTKView: MTKView, MTKViewDelegate {
    public var dataSource: MJRenderDataSource?

    private var renderer: MJMetalRenderer?

    // Performance metrics
    private var frameCount: Int = 0
    private var lastFPSUpdate: CFTimeInterval = CACurrentMediaTime()
    public private(set) var renderFPS: Double = 0
    public private(set) var lastFrameTime: Double = 0  // ms

    // Touch tracking
    #if os(iOS)
    private var lastPanLocation: CGPoint = .zero
    private var lastPinchScale: CGFloat = 1.0
    #endif

    public override init(frame: CGRect, device: MTLDevice?) {
        super.init(frame: frame, device: device ?? MTLCreateSystemDefaultDevice())
        commonInit()
    }

    public required init(coder: NSCoder) {
        super.init(coder: coder)
        commonInit()
    }

    private func commonInit() {
        guard let device = self.device else {
            print("Metal is not supported on this device")
            return
        }

        self.delegate = self
        self.colorPixelFormat = .bgra8Unorm
        self.depthStencilPixelFormat = .depth32Float
        self.preferredFramesPerSecond = 60
        self.enableSetNeedsDisplay = false
        self.isPaused = false

        // Initialize Metal renderer
        do {
            renderer = try MJMetalRenderer(device: device)
        } catch {
            print("Failed to create Metal renderer: \(error)")
        }

        #if os(iOS)
        setupGestures()
        #endif

        #if os(tvOS)
        setupTVGestures()
        #endif
    }

    #if os(iOS)
    private func setupGestures() {
        // Pan for camera rotation
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
        addGestureRecognizer(panGesture)

        // Pinch for zoom
        let pinchGesture = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        addGestureRecognizer(pinchGesture)

        // Double tap to reset camera
        let doubleTapGesture = UITapGestureRecognizer(target: self, action: #selector(handleDoubleTap(_:)))
        doubleTapGesture.numberOfTapsRequired = 2
        addGestureRecognizer(doubleTapGesture)
    }

    @objc private func handlePan(_ gesture: UIPanGestureRecognizer) {
        guard let dataSource = dataSource else { return }

        let location = gesture.translation(in: self)

        if gesture.state == .began {
            lastPanLocation = .zero
        }

        let deltaX = location.x - lastPanLocation.x
        let deltaY = location.y - lastPanLocation.y

        // Update camera azimuth and elevation
        dataSource.cameraAzimuth += Double(deltaX) * 0.5
        dataSource.cameraElevation += Double(deltaY) * 0.5

        lastPanLocation = location
    }

    @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
        guard let dataSource = dataSource else { return }

        if gesture.state == .began {
            lastPinchScale = 1.0
        }

        let scale = gesture.scale / lastPinchScale
        guard scale > 0 else { return }

        let newDistance = dataSource.cameraDistance * (1.0 / Double(scale))
        dataSource.cameraDistance = max(newDistance, minCameraDistance)

        lastPinchScale = gesture.scale
    }

    @objc private func handleDoubleTap(_ gesture: UITapGestureRecognizer) {
        dataSource?.resetCamera()
    }
    #endif

    #if os(tvOS)
    private var lastSwipeLocation: CGPoint = .zero

    private func setupTVGestures() {
        // Swipe/pan for camera rotation (works with Siri Remote touch surface)
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(handleTVPan(_:)))
        panGesture.allowedTouchTypes = [NSNumber(value: UITouch.TouchType.indirect.rawValue)]
        addGestureRecognizer(panGesture)

        // Tap to reset camera (using press gesture)
        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleTVTap(_:)))
        tapGesture.allowedPressTypes = [NSNumber(value: UIPress.PressType.select.rawValue)]
        addGestureRecognizer(tapGesture)
    }

    @objc private func handleTVPan(_ gesture: UIPanGestureRecognizer) {
        guard let dataSource = dataSource else { return }

        let velocity = gesture.velocity(in: self)

        // Swipe sensitivity for tvOS remote
        let sensitivity: CGFloat = 0.02

        // Horizontal swipe = azimuth rotation, vertical swipe = elevation
        dataSource.cameraAzimuth += Double(velocity.x) * sensitivity
        dataSource.cameraElevation += Double(velocity.y) * sensitivity

        // Use up/down for zoom when swiping near edges or with specific gesture
        if abs(velocity.y) > 500 && abs(velocity.x) < 100 {
            let zoomFactor = velocity.y > 0 ? 1.02 : 0.98
            let newDistance = dataSource.cameraDistance * zoomFactor
            dataSource.cameraDistance = max(newDistance, minCameraDistance)
        }
    }

    @objc private func handleTVTap(_ gesture: UITapGestureRecognizer) {
        dataSource?.resetCamera()
    }
    #endif

    #if os(macOS)
    // MARK: - macOS Mouse Handling

    private var lastMouseLocation: CGPoint = .zero
    private var isRotating = false

    public override var acceptsFirstResponder: Bool { true }

    public override func mouseDown(with event: NSEvent) {
        lastMouseLocation = event.locationInWindow
        isRotating = true
    }

    public override func mouseUp(with event: NSEvent) {
        isRotating = false
    }

    public override func mouseDragged(with event: NSEvent) {
        guard let dataSource = dataSource, isRotating else { return }

        let location = event.locationInWindow
        let deltaX = location.x - lastMouseLocation.x
        let deltaY = location.y - lastMouseLocation.y

        // Update camera azimuth and elevation
        dataSource.cameraAzimuth += Double(deltaX) * 0.5
        dataSource.cameraElevation -= Double(deltaY) * 0.5  // Inverted for natural feel

        lastMouseLocation = location
    }

    public override func rightMouseDragged(with event: NSEvent) {
        guard let dataSource = dataSource else { return }

        // Right drag for zoom
        let deltaY = event.deltaY
        let newDistance = dataSource.cameraDistance * (1.0 + Double(deltaY) * 0.01)
        dataSource.cameraDistance = max(newDistance, minCameraDistance)
    }

    public override func scrollWheel(with event: NSEvent) {
        guard let dataSource = dataSource else { return }

        // Scroll wheel for zoom
        let deltaY = event.scrollingDeltaY
        let newDistance = dataSource.cameraDistance * (1.0 - Double(deltaY) * 0.01)
        dataSource.cameraDistance = max(newDistance, minCameraDistance)
    }

    public override func keyDown(with event: NSEvent) {
        // 'R' key to reset camera
        if event.charactersIgnoringModifiers == "r" {
            dataSource?.resetCamera()
        }
    }
    #endif

    // MARK: - MTKViewDelegate

    public func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        // No action needed - renderer handles size changes dynamically
    }

    public func draw(in view: MTKView) {
        let frameStart = CACurrentMediaTime()

        guard let renderer = renderer,
              let dataSource = dataSource else {
            return
        }

        // Get current drawable
        guard let drawable = currentDrawable else { return }

        // Render using lock-free ring buffer API (preferred)
        if let frameData = dataSource.latestFrame {
            renderer.render(
                frameData: frameData,
                drawable: drawable,
                renderPassDescriptor: currentRenderPassDescriptor
            )
        }
        // Fallback to legacy scene API if ring buffer not available
        else if let scene = dataSource.scenePointer,
                let camera = dataSource.cameraPointer {
            renderer.render(
                scene: scene,
                camera: camera,
                drawable: drawable,
                renderPassDescriptor: currentRenderPassDescriptor
            )
        }

        // Update performance metrics
        let frameEnd = CACurrentMediaTime()
        lastFrameTime = (frameEnd - frameStart) * 1000.0  // Convert to ms

        frameCount += 1
        let elapsed = frameEnd - lastFPSUpdate
        if elapsed >= 1.0 {
            renderFPS = Double(frameCount) / elapsed
            frameCount = 0
            lastFPSUpdate = frameEnd
        }
    }
}
