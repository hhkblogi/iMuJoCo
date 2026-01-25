import Foundation
import MJCPhysicsRuntime

/// Core framework for iMuJoCo
/// Provides Swift interface to MuJoCo physics simulation

public enum MuJoCo {
    /// Get MuJoCo version number (e.g., 340 for version 3.4.0)
    public static var Version: Int32 {
        return MJGetVersion()
    }

    /// Get MuJoCo version string (e.g., "3.4.0")
    public static var VersionString: String {
        let v = MJGetVersion()
        let major = v / 100
        let minor = (v % 100) / 10
        let patch = v % 10
        return "\(major).\(minor).\(patch)"
    }
}
