// mujoco_error.swift
// MuJoCo error types

import Foundation

// MARK: - MuJoCo Error

enum MuJoCoError: Error, LocalizedError {
    case loadFailed(String)
    case dataCreationFailed
    case invalidModel

    var errorDescription: String? {
        switch self {
        case .loadFailed(let message): return "Failed to load model: \(message)"
        case .dataCreationFailed: return "Failed to create simulation data"
        case .invalidModel: return "Invalid model"
        }
    }
}
