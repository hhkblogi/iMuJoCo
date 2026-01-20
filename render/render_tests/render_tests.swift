// render_tests.swift
// Unit tests for the render framework

import XCTest
import simd
import Metal
@testable import render

// MARK: - Matrix Math Tests

final class MatrixMathTests: XCTestCase {

    func test_simd_float4x4_identity() {
        let identity = simd_float4x4(1.0)

        XCTAssertEqual(identity.columns.0.x, 1.0)
        XCTAssertEqual(identity.columns.1.y, 1.0)
        XCTAssertEqual(identity.columns.2.z, 1.0)
        XCTAssertEqual(identity.columns.3.w, 1.0)

        // Off-diagonal should be zero
        XCTAssertEqual(identity.columns.0.y, 0.0)
        XCTAssertEqual(identity.columns.1.x, 0.0)
    }
}

// MARK: - Vertex Layout Tests

final class VertexLayoutTests: XCTestCase {

    func test_vertex_struct_size() {
        // MJCMetalVertex is designed to be efficiently consumed by the GPU.
        // A 64-byte stride is typical for optimal alignment, but the key requirements are:
        // 1. At least 64 bytes to hold all vertex data (position, normal, texCoord, color)
        // 2. Multiple of 16 bytes for proper GPU alignment
        let stride = MemoryLayout<MJCMetalVertex>.stride
        XCTAssertGreaterThanOrEqual(stride, 64, "Expected vertex stride to be at least 64 bytes, got \(stride)")
        XCTAssertEqual(stride % 16, 0, "Expected vertex stride to be a multiple of 16 bytes for alignment, got \(stride)")
    }

    func test_vertex_struct_alignment() {
        // Verify 16-byte alignment for GPU compatibility
        let alignment = MemoryLayout<MJCMetalVertex>.alignment
        XCTAssertGreaterThanOrEqual(alignment, 16, "Expected at least 16-byte alignment")
    }
}

// MARK: - Uniforms Layout Tests

final class UniformsLayoutTests: XCTestCase {

    func test_uniforms_struct_alignment() {
        // Uniforms need proper alignment for Metal buffer binding
        let alignment = MemoryLayout<MJCMetalUniforms>.alignment
        XCTAssertGreaterThanOrEqual(alignment, 16, "Expected at least 16-byte alignment for uniforms")
    }

    func test_uniforms_struct_size() {
        // Verify struct size is at least the minimum expected layout:
        // - 4 x float4x4 (4 * 64 = 256 bytes)
        // - lightPosition float3 + padding (16 bytes)
        // - cameraPosition float3 + padding (16 bytes)
        // - color float4 (16 bytes)
        // - emission, specular, shininess, padding (16 bytes)
        // Minimum expected: 320 bytes (but may be larger due to alignment)
        let stride = MemoryLayout<MJCMetalUniforms>.stride
        XCTAssertGreaterThanOrEqual(stride, 320, "Expected uniforms stride to be at least 320 bytes, got \(stride)")
        XCTAssertEqual(stride % 16, 0, "Expected uniforms stride to be a multiple of 16 bytes for alignment, got \(stride)")
    }
}

// MARK: - Renderer Creation Tests

final class RendererCreationTests: XCTestCase {

    func test_renderer_creation_with_device() {
        // Creating a renderer requires a valid Metal device
        guard let device = MTLCreateSystemDefaultDevice() else {
            // Metal not available (e.g., in CI without GPU)
            return
        }

        do {
            let renderer = try MJCMetalRender(device: device)
            XCTAssertNotNil(renderer)
        } catch {
            XCTFail("Failed to create renderer: \(error)")
        }
    }
}
