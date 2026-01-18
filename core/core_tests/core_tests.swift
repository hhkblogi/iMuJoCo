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

