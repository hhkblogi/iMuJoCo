import Testing
@testable import core

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

}

