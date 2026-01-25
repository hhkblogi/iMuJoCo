import Testing
@testable import core
import MJCPhysicsRuntime

struct CoreTests {

    @Test func test_mujoco_version() async throws {
        // Verify MuJoCo is linked correctly by checking version
        let version = MuJoCo.Version
        #expect(version == 340, "Expected MuJoCo version 340 (3.4.0), got \(version)")
    }

    @Test func test_mujoco_version_string() async throws {
        // Verify version string formatting
        let version_string = MuJoCo.VersionString
        #expect(version_string == "3.4.0", "Expected version string '3.4.0', got '\(version_string)'")
    }

}

// MARK: - MJRuntime Tests

struct MJRuntimeTests {

    @Test func test_runtime_creation() async throws {
        // Verify runtime can be created
        let runtime = try MJRuntime(instanceIndex: 0)
        #expect(runtime.state == .inactive)
    }

    @Test func test_runtime_invalid_model() async throws {
        // Verify error handling for invalid model
        let runtime = try MJRuntime(instanceIndex: 1)

        do {
            try runtime.loadModel(fromXML: "invalid xml content")
            Issue.record("Expected loadModel to throw for invalid XML")
        } catch MJRuntimeError.loadFailed {
            // Expected
        }

        #expect(runtime.state == .inactive)
    }

    @Test func test_runtime_load_simple_model() async throws {
        // Test loading a simple valid MuJoCo model
        let runtime = try MJRuntime(instanceIndex: 2)

        let simpleModel = """
        <mujoco>
          <worldbody>
            <body name="box">
              <geom type="box" size="0.1 0.1 0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """

        try runtime.loadModel(fromXML: simpleModel)
        #expect(runtime.state == .loaded)
        #expect(runtime.timestep > 0)
    }

    @Test func test_runtime_state_transitions() async throws {
        // Test start/pause/reset state transitions
        let runtime = try MJRuntime(instanceIndex: 3)

        let simpleModel = """
        <mujoco>
          <worldbody>
            <body name="box">
              <geom type="box" size="0.1 0.1 0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """

        try runtime.loadModel(fromXML: simpleModel)
        #expect(runtime.state == .loaded)

        runtime.start()
        // Give physics thread time to start
        try await Task.sleep(for: .milliseconds(50))
        #expect(runtime.state == .running)

        runtime.pause()
        // Give physics thread time to pause
        try await Task.sleep(for: .milliseconds(50))
        #expect(runtime.state == .paused)

        runtime.reset()
        #expect(runtime.simulationTime < 0.1) // Should be near zero after reset
    }

    @Test func test_runtime_frame_counter() async throws {
        // Test frame count increments during simulation
        let runtime = try MJRuntime(instanceIndex: 4)

        let simpleModel = """
        <mujoco>
          <worldbody>
            <body name="box">
              <geom type="box" size="0.1 0.1 0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """

        try runtime.loadModel(fromXML: simpleModel)

        // Capture initial frame count before starting to avoid race condition
        let initialFrameCount = runtime.frameCount

        runtime.start()

        // Poll for frame count increase with timeout
        let startTime = ContinuousClock.now
        let timeout: Duration = .milliseconds(500)
        var frameCountIncreased = false

        while ContinuousClock.now - startTime < timeout {
            try await Task.sleep(for: .milliseconds(10))
            if runtime.frameCount > initialFrameCount {
                frameCountIncreased = true
                break
            }
        }

        // frameCount should have increased
        #expect(frameCountIncreased, "Frame count should increase from \(initialFrameCount) to \(runtime.frameCount)")

        runtime.pause()
    }

    @Test func test_runtime_frame_data_access() async throws {
        // Test accessing frame data through latestFrame and getGeom()
        // This verifies Swift C++ interop works correctly for MJFrameData access
        // MJFrameData is now a SWIFT_IMMORTAL_REFERENCE class - reference semantics, no copying
        let runtime = try MJRuntime(instanceIndex: 5)

        let modelWithGeom = """
        <mujoco>
          <worldbody>
            <body name="box" pos="0 0 1">
              <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
            </body>
          </worldbody>
        </mujoco>
        """

        try runtime.loadModel(fromXML: modelWithGeom)
        runtime.start()

        // Wait for frames to be available
        try await Task.sleep(for: .milliseconds(200))

        // Get latest frame - MJFrameData is a reference type (class)
        guard let frame = runtime.latestFrame else {
            Issue.record("latestFrame returned nil")
            runtime.pause()
            return
        }

        // Access geom count through class method
        let geomCount = frame.geomCount()
        #expect(geomCount > 0, "Frame should have at least one geom (got \(geomCount))")

        // Access camera data through class methods
        let cameraDistance = frame.cameraDistance()
        #expect(cameraDistance > 0, "Camera distance should be positive")

        // Access geom through free function (member functions returning pointers not supported)
        if geomCount > 0 {
            if let geomsPtr = MJFrameDataGetGeoms(frame) {
                let geom = geomsPtr[0]
                // Verify geom has valid type (box = 6 in MuJoCo)
                #expect(geom.type >= 0, "Geom type should be non-negative (got \(geom.type))")
            } else {
                Issue.record("geoms() returned nil")
            }
        }

        runtime.pause()
    }

    @Test func test_runtime_camera_control() async throws {
        // Test camera control methods
        let runtime = try MJRuntime(instanceIndex: 6)

        let simpleModel = """
        <mujoco>
          <worldbody>
            <body name="box">
              <geom type="box" size="0.1 0.1 0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """

        try runtime.loadModel(fromXML: simpleModel)

        // Test camera setters
        runtime.cameraAzimuth = 45.0
        runtime.cameraElevation = -30.0
        runtime.cameraDistance = 5.0
        runtime.setCameraLookat(x: 1.0, y: 2.0, z: 0.5)

        // Verify values were set
        #expect(Swift.abs(runtime.cameraAzimuth - 45.0) < 0.01)
        #expect(Swift.abs(runtime.cameraElevation - (-30.0)) < 0.01)
        #expect(Swift.abs(runtime.cameraDistance - 5.0) < 0.01)

        // Test reset
        runtime.resetCamera()
    }

}

