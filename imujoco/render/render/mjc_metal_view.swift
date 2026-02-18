// mjc_metal_view.swift
// SwiftUI wrapper for Metal-based MuJoCo rendering

import SwiftUI
import MetalKit
import os.log
import MJCPhysicsRuntime

private let logger = Logger(subsystem: "com.mujoco.render", category: "MuJoCoMTKView")

// MARK: - Runtime Protocol

/// Protocol for physics runtime that provides frame data for rendering.
///
/// Implement this protocol to connect your physics runtime to the Metal view.
/// The render will call `latestFrame` each frame to get geometry data for rendering.
///
/// - Note: This protocol uses MJFrameData (C++ interop) instead of raw pointers.
///   The legacy scenePointer/cameraPointer API has been removed in favor of the
///   safer, typed MJFrameData interface.
///
/// ## Camera Control
/// The protocol includes camera properties that gesture handlers will update.
/// Your implementation should apply these values to the underlying camera system.
///
/// ## Example Implementation
/// ```swift
/// class MyRuntime: MJCRenderDataSource {
///     var latestFrame: MJFrameData? {
///         return source.latestFrame
///     }
///     // ... implement other requirements
/// }
///
/// // Safe usage - access frame properties within a single scope:
/// if let frame = runtime.latestFrame {
///     let count = frame.geomCount()
///     // Use frame data here - do not store beyond this scope
/// }
/// ```
public protocol MJCRenderDataSource: AnyObject {
    /// Get the latest frame data for rendering (non-blocking).
    /// Returns nil if no frame is available yet.
    ///
    /// - Important: The returned frame is only valid until the next `latestFrame` call
    ///   on the same thread. Do not store references across calls.
    var latestFrame: MJFrameData? { get }

    /// Get pre-loaded mesh data for rendering (available after model load).
    /// Returns nil if no model is loaded or model has no meshes.
    var meshData: MJMeshData? { get }

    /// Camera azimuth angle in degrees (horizontal rotation).
    var cameraAzimuth: Double { get set }

    /// Camera elevation angle in degrees (vertical rotation).
    var cameraElevation: Double { get set }

    /// Camera distance from the lookat point.
    /// Must remain positive; gesture handlers clamp to `min_camera_distance`.
    var cameraDistance: Double { get set }

    /// Camera lookat point X coordinate (world space).
    var cameraLookatX: Double { get set }

    /// Camera lookat point Y coordinate (world space).
    var cameraLookatY: Double { get set }

    /// Camera lookat point Z coordinate (world space).
    var cameraLookatZ: Double { get set }

    /// Average rendered scene brightness (0.0 dark – 1.0 bright).
    /// Written by the render thread after each frame via GPU pixel readback.
    var renderedSceneBrightness: Float { get set }

    /// Reset camera to default position.
    func resetCamera()
}

/// Minimum camera distance to prevent rendering issues from zero/negative distances.
private let min_camera_distance: Double = 0.1

/// Camera rotation sensitivity (degrees per point of drag/pan movement).
private let camera_rotation_sensitivity: Double = 0.5

/// Zoom sensitivity for macOS scroll/drag (percentage change per point).
private let zoom_sensitivity: Double = 0.01

/// tvOS zoom factor per high-velocity vertical swipe (2% change).
private let tvos_zoom_factor: Double = 0.02

/// tvOS velocity thresholds for zoom detection (points per second).
private let tvos_zoom_velocity_threshold: CGFloat = 500
private let tvos_zoom_horizontal_limit: CGFloat = 100

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
    public var dataSource: MJCRenderDataSource

    /// Whether this view is displayed in fullscreen mode (affects gizmo positioning).
    public var isFullscreen: Bool

    /// Creates a new MuJoCo Metal view with the specified data source.
    public init(dataSource: MJCRenderDataSource, isFullscreen: Bool = false) {
        self.dataSource = dataSource
        self.isFullscreen = isFullscreen
    }

    public func makeUIView(context: Context) -> MuJoCoMTKView {
        let view = MuJoCoMTKView()
        view.dataSource = dataSource
        view.isFullscreen = isFullscreen
        return view
    }

    public func updateUIView(_ uiView: MuJoCoMTKView, context: Context) {
        uiView.dataSource = dataSource
        uiView.isFullscreen = isFullscreen
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
    public var dataSource: MJCRenderDataSource

    /// Whether this view is displayed in fullscreen mode (affects gizmo positioning).
    public var isFullscreen: Bool

    /// Creates a new MuJoCo Metal view with the specified data source.
    public init(dataSource: MJCRenderDataSource, isFullscreen: Bool = false) {
        self.dataSource = dataSource
        self.isFullscreen = isFullscreen
    }

    public func makeNSView(context: Context) -> MuJoCoMTKView {
        let view = MuJoCoMTKView()
        view.dataSource = dataSource
        view.isFullscreen = isFullscreen
        return view
    }

    public func updateNSView(_ nsView: MuJoCoMTKView, context: Context) {
        nsView.dataSource = dataSource
        nsView.isFullscreen = isFullscreen
    }
}
#endif

// MARK: - Custom MTKView

public class MuJoCoMTKView: MTKView, MTKViewDelegate {
    public var dataSource: MJCRenderDataSource? {
        didSet {
            // Start rendering when dataSource is set (if everything else is ready)
            check_ready_to_render()
        }
    }

    /// Whether this view is displayed in fullscreen mode (affects gizmo positioning).
    public var isFullscreen: Bool = false

    private var render: MJCMetalRender?

    // Track if view is ready for rendering (avoids crashes during deallocation)
    private var isRenderingEnabled: Bool = true

    // Lock to prevent deallocation during rendering
    private let renderLock = NSLock()

    // Performance metrics
    private var frame_count: Int = 0
    private var last_fps_update: CFTimeInterval = CACurrentMediaTime()
    public private(set) var renderFPS: Double = 0
    public private(set) var lastFrameTime: Double = 0  // ms

    // Touch tracking
    #if os(iOS)
    private var last_pan_location: CGPoint = .zero
    private var last_translate_location: CGPoint = .zero
    private var last_pinch_scale: CGFloat = 1.0
    #endif

    public override init(frame: CGRect, device: MTLDevice?) {
        super.init(frame: frame, device: device ?? MTLCreateSystemDefaultDevice())
        common_init()
    }

    public required init(coder: NSCoder) {
        super.init(coder: coder)
        common_init()
    }

    deinit {
        // Lifecycle shutdown sequence:
        // 1. isPaused = true prevents Metal from scheduling new draw calls
        // 2. lock() waits for any in-progress draw() to complete (if it holds the lock)
        // 3. isRenderingEnabled = false ensures any draw() that passed the first check
        //    will exit at the double-check after acquiring the lock
        //
        // Note: deinit may block briefly if draw() is in progress. This is a deliberate
        // tradeoff - we ensure clean shutdown rather than racing with the render thread.
        // In practice draw() completes quickly (<16ms per frame). If draw() were to hang,
        // deinit would also hang, but this indicates a deeper Metal/GPU issue.
        isPaused = true
        renderLock.lock()
        isRenderingEnabled = false
        delegate = nil
        render = nil
        renderLock.unlock()
    }

    private func common_init() {
        guard let device = self.device else {
            logger.error("Metal is not supported on this device")
            isRenderingEnabled = false
            return
        }

        // Keep paused during initialization
        self.isPaused = true
        self.colorPixelFormat = .bgra8Unorm
        self.depthStencilPixelFormat = .depth32Float
        self.framebufferOnly = false  // Allow blit readback for brightness sampling
        self.preferredFramesPerSecond = 60
        self.enableSetNeedsDisplay = false

        // Initialize Metal render BEFORE setting delegate
        // This ensures render is ready before any draw calls
        do {
            render = try MJCMetalRender(device: device)
        } catch {
            logger.error("Failed to create Metal render: \(error)")
            isRenderingEnabled = false
            return
        }

        #if os(iOS)
        setup_gestures()
        #endif

        #if os(tvOS)
        setup_tv_gestures()
        #endif

        #if os(macOS)
        // No special touch types needed; pan uses Option+scroll instead of 3-finger touches
        #endif

        // Set delegate last, after everything is fully initialized
        self.delegate = self

        // Only enable rendering after everything is initialized
        // Note: We DON'T unpause here - we wait for dataSource to be set
        isRenderingEnabled = true
    }

    // Called when dataSource is set - this is when we can safely start rendering
    private func check_ready_to_render() {
        if isRenderingEnabled && render != nil && dataSource != nil && isPaused {
            isPaused = false
        }
    }

    #if os(iOS)
    private func setup_gestures() {
        // One-finger pan for camera rotation
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(handle_pan(_:)))
        panGesture.minimumNumberOfTouches = 1
        panGesture.maximumNumberOfTouches = 1
        addGestureRecognizer(panGesture)

        // Three-finger pan for camera translation
        let translateGesture = UIPanGestureRecognizer(target: self, action: #selector(handle_translate(_:)))
        translateGesture.minimumNumberOfTouches = 3
        translateGesture.maximumNumberOfTouches = 3
        addGestureRecognizer(translateGesture)

        // Pinch for zoom
        let pinchGesture = UIPinchGestureRecognizer(target: self, action: #selector(handle_pinch(_:)))
        addGestureRecognizer(pinchGesture)

    }

    @objc private func handle_pan(_ gesture: UIPanGestureRecognizer) {
        guard let dataSource = dataSource else { return }

        let location = gesture.translation(in: self)

        if gesture.state == .began {
            // Store initial translation to avoid jump on first movement
            last_pan_location = location
        }

        let deltaX = location.x - last_pan_location.x
        let deltaY = location.y - last_pan_location.y

        // Update camera azimuth and elevation
        dataSource.cameraAzimuth += Double(deltaX) * camera_rotation_sensitivity
        dataSource.cameraElevation += Double(deltaY) * camera_rotation_sensitivity

        last_pan_location = location
    }

    @objc private func handle_translate(_ gesture: UIPanGestureRecognizer) {
        guard let dataSource = dataSource else { return }

        let location = gesture.translation(in: self)

        if gesture.state == .began {
            last_translate_location = location
        }

        let deltaX = Double(location.x - last_translate_location.x)
        let deltaY = Double(location.y - last_translate_location.y)
        translate_camera(dataSource: dataSource, deltaX: deltaX, deltaY: -deltaY)
        last_translate_location = location
    }

    @objc private func handle_pinch(_ gesture: UIPinchGestureRecognizer) {
        guard let dataSource = dataSource else { return }

        switch gesture.state {
        case .began:
            // Initialize reference scale for this gesture sequence
            last_pinch_scale = gesture.scale

        case .changed:
            // Compute scale delta relative to previous reading
            guard last_pinch_scale > 0 else {
                last_pinch_scale = gesture.scale
                return
            }

            let scaleDelta = gesture.scale / last_pinch_scale
            guard scaleDelta > 0 else {
                last_pinch_scale = gesture.scale
                return
            }

            let newDistance = dataSource.cameraDistance * (1.0 / Double(scaleDelta))
            dataSource.cameraDistance = max(newDistance, min_camera_distance)

            // Store current scale for next delta computation
            last_pinch_scale = gesture.scale

        case .ended, .cancelled, .failed:
            // Reset between gesture sequences to avoid cross-gesture error accumulation
            last_pinch_scale = 1.0

        default:
            break
        }
    }

    #endif

    #if os(tvOS)
    private func setup_tv_gestures() {
        // Swipe/pan for camera rotation (works with Siri Remote touch surface)
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(handle_tv_pan(_:)))
        panGesture.allowedTouchTypes = [NSNumber(value: UITouch.TouchType.indirect.rawValue)]
        addGestureRecognizer(panGesture)

        // Tap to reset camera (using press gesture)
        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handle_tv_tap(_:)))
        tapGesture.allowedPressTypes = [NSNumber(value: UIPress.PressType.select.rawValue)]
        addGestureRecognizer(tapGesture)
    }

    @objc private func handle_tv_pan(_ gesture: UIPanGestureRecognizer) {
        guard let dataSource = dataSource else { return }

        let velocity = gesture.velocity(in: self)

        // Swipe sensitivity for tvOS remote (velocity is in points per second)
        let sensitivity: CGFloat = 0.02

        // Horizontal swipe = azimuth rotation, vertical swipe = elevation
        dataSource.cameraAzimuth += Double(velocity.x) * sensitivity
        dataSource.cameraElevation += Double(velocity.y) * sensitivity

        // Detect high-velocity vertical swipe for zoom (primarily vertical movement)
        if abs(velocity.y) > tvos_zoom_velocity_threshold && abs(velocity.x) < tvos_zoom_horizontal_limit {
            let zoomFactor = 1.0 + (velocity.y > 0 ? tvos_zoom_factor : -tvos_zoom_factor)
            let newDistance = dataSource.cameraDistance * zoomFactor
            dataSource.cameraDistance = max(newDistance, min_camera_distance)
        }
    }

    @objc private func handle_tv_tap(_ gesture: UITapGestureRecognizer) {
        dataSource?.resetCamera()
    }
    #endif

    #if os(macOS)
    // MARK: - macOS Mouse Handling

    private var last_mouse_location: CGPoint = .zero
    private var is_rotating = false

    public override var acceptsFirstResponder: Bool { true }

    public override func mouseDown(with event: NSEvent) {
        last_mouse_location = event.locationInWindow
        is_rotating = true
    }

    public override func mouseUp(with event: NSEvent) {
        is_rotating = false
    }

    public override func mouseDragged(with event: NSEvent) {
        guard let dataSource = dataSource, is_rotating else { return }

        let location = event.locationInWindow
        let deltaX = location.x - last_mouse_location.x
        let deltaY = location.y - last_mouse_location.y

        // Update camera azimuth and elevation
        dataSource.cameraAzimuth += Double(deltaX) * camera_rotation_sensitivity
        dataSource.cameraElevation -= Double(deltaY) * camera_rotation_sensitivity  // Inverted for natural feel

        last_mouse_location = location
    }

    public override func rightMouseDragged(with event: NSEvent) {
        guard let dataSource = dataSource else { return }

        // Right drag for zoom
        let deltaY = event.deltaY
        let newDistance = dataSource.cameraDistance * (1.0 + Double(deltaY) * zoom_sensitivity)
        dataSource.cameraDistance = max(newDistance, min_camera_distance)
    }

    public override func scrollWheel(with event: NSEvent) {
        guard let dataSource = dataSource else { return }

        if event.modifierFlags.contains(.option) {
            // Option + two-finger scroll → pan (translate camera lookat)
            let deltaX = Double(event.scrollingDeltaX)
            let deltaY = Double(event.scrollingDeltaY)
            translate_camera(dataSource: dataSource, deltaX: deltaX, deltaY: deltaY)
        } else {
            // Two-finger scroll → zoom
            let deltaY = event.scrollingDeltaY
            let newDistance = dataSource.cameraDistance * (1.0 - Double(deltaY) * zoom_sensitivity)
            dataSource.cameraDistance = max(newDistance, min_camera_distance)
        }
    }

    public override func magnify(with event: NSEvent) {
        guard let dataSource = dataSource else { return }

        // Trackpad pinch → zoom
        let newDistance = dataSource.cameraDistance * (1.0 - Double(event.magnification))
        dataSource.cameraDistance = max(newDistance, min_camera_distance)
    }

    public override func keyDown(with event: NSEvent) {
        // 'R' key to reset camera
        if event.charactersIgnoringModifiers == "r" {
            dataSource?.resetCamera()
        }
    }
    #endif

    // MARK: - Camera Translation

    /// Pan sensitivity (screen points to world units, scaled by camera distance).
    private let pan_sensitivity: Double = 0.003

    /// Translate camera lookat point based on screen-space deltas.
    /// Positive deltaX moves the scene right, positive deltaY moves it up.
    private func translate_camera(dataSource: MJCRenderDataSource, deltaX: Double, deltaY: Double) {
        let az = dataSource.cameraAzimuth * .pi / 180.0
        let el = dataSource.cameraElevation * .pi / 180.0
        let scale = dataSource.cameraDistance * pan_sensitivity

        // Right vector (screen X → world)
        let rx = cos(az), ry = -sin(az)
        // Up vector (screen Y → world)
        let ux = -sin(el) * sin(az), uy = -sin(el) * cos(az), uz = cos(el)

        dataSource.cameraLookatX += deltaX * rx * scale - deltaY * ux * scale
        dataSource.cameraLookatY += deltaX * ry * scale - deltaY * uy * scale
        dataSource.cameraLookatZ -= deltaY * uz * scale
    }

    // MARK: - MTKViewDelegate

    public func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        // No action needed - render handles size changes dynamically
    }

    public func draw(in view: MTKView) {
        // Early exit if rendering is disabled (during deallocation or failed init)
        guard isRenderingEnabled else { return }

        // Acquire lock to prevent deallocation during rendering
        guard renderLock.try() else { return }
        defer { renderLock.unlock() }

        // Double-check after acquiring lock
        guard isRenderingEnabled else { return }

        let frameStart = CACurrentMediaTime()

        // Capture strong references to ensure objects survive the entire method
        guard let render = self.render,
              let dataSource = self.dataSource else {
            return
        }

        // Get current drawable - may be nil if view is not visible
        guard let drawable = currentDrawable else { return }

        // Get frame data from the data source
        // Note: latestFrame may return nil during startup or if no frames have been written
        // IMPORTANT: MJFrameData is only valid until the next latestFrame call on this thread.
        // Do not store the frame reference beyond this draw call.
        guard let frame = dataSource.latestFrame else {
            // No frame available yet - this is normal during startup
            return
        }

        // Only render if we have geometry to draw
        let geomCount = frame.geomCount()
        if geomCount > 0 {
            render.Render(
                frame: frame,
                meshData: dataSource.meshData,
                drawable: drawable,
                renderPassDescriptor: currentRenderPassDescriptor,
                isFullscreen: isFullscreen
            )

            // Pass GPU-computed brightness back to data source for UI overlay adaptation
            dataSource.renderedSceneBrightness = render.renderedBrightness
        }

        // Update performance metrics
        let frameEnd = CACurrentMediaTime()
        lastFrameTime = (frameEnd - frameStart) * 1000.0  // Convert to ms

        frame_count += 1
        let elapsed = frameEnd - last_fps_update
        if elapsed >= 1.0 {
            renderFPS = Double(frame_count) / elapsed
            frame_count = 0
            last_fps_update = frameEnd
        }
    }
}
