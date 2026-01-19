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
        // MJMetalVertex should be 64 bytes for optimal GPU alignment
        let stride = MemoryLayout<MJMetalVertex>.stride
        XCTAssertEqual(stride, 64, "Expected 64-byte vertex stride, got \(stride)")
    }

    func test_vertex_struct_alignment() {
        // Verify 16-byte alignment for GPU compatibility
        let alignment = MemoryLayout<MJMetalVertex>.alignment
        XCTAssertGreaterThanOrEqual(alignment, 16, "Expected at least 16-byte alignment")
    }
}

// MARK: - Uniforms Layout Tests

final class UniformsLayoutTests: XCTestCase {

    func test_uniforms_struct_alignment() {
        // Uniforms need proper alignment for Metal buffer binding
        let alignment = MemoryLayout<MJMetalUniforms>.alignment
        XCTAssertGreaterThanOrEqual(alignment, 16, "Expected at least 16-byte alignment for uniforms")
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
            let renderer = try MJMetalRenderer(device: device)
            XCTAssertNotNil(renderer)
        } catch {
            XCTFail("Failed to create renderer: \(error)")
        }
    }
}
