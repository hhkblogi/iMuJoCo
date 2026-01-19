// mj_metal_renderer.swift
// Pure Swift Metal renderer for MuJoCo scenes

import Metal
import MetalKit
import simd
import MJCPhysicsRuntime

// MARK: - Shader Types

struct MJMetalUniforms {
    var modelMatrix: simd_float4x4
    var viewMatrix: simd_float4x4
    var projectionMatrix: simd_float4x4
    var normalMatrix: simd_float4x4
    var lightPosition: simd_float3
    var _padding0: Float = 0  // Alignment padding after lightPosition (float3 aligns to 16 bytes in Metal)
    var cameraPosition: simd_float3
    var _padding1: Float = 0  // Alignment padding after cameraPosition
    var color: simd_float4
    var emission: Float
    var specular: Float
    var shininess: Float
    var _padding2: Float = 0  // Final alignment padding
}

struct MJMetalVertex {
    var position: simd_float3      // offset 0, size 12, padded to 16
    var normal: simd_float3        // offset 16, size 12, padded to 16
    var texCoord: simd_float2      // offset 32, size 8
    var _padding: simd_float2 = .zero  // offset 40, size 8 - pad to 16-byte boundary
    var color: simd_float4         // offset 48, size 16
    // Total: 64 bytes
}

// MARK: - Metal Renderer

public final class MJMetalRenderer {
    private let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private let pipelineState: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState

    private var vertexBuffer: MTLBuffer
    private var indexBuffer: MTLBuffer
    private var depthTexture: MTLTexture?

    private let maxVertices = 1024 * 1024
    private let maxIndices = 1024 * 1024

    // MARK: - Initialization

    public init(device: MTLDevice) throws {
        self.device = device

        guard let queue = device.makeCommandQueue() else {
            throw MJRendererError.commandQueueCreationFailed
        }
        self.commandQueue = queue

        // Load shaders from compiled Metal library in the framework bundle
        let bundle = Bundle(for: MJMetalRenderer.self)
        guard let library = try? device.makeDefaultLibrary(bundle: bundle) else {
            throw MJRendererError.shaderCompilationFailed
        }

        guard let vertexFunc = library.makeFunction(name: "vertexMain"),
              let fragmentFunc = library.makeFunction(name: "fragmentMain") else {
            throw MJRendererError.shaderFunctionNotFound
        }

        // Create vertex descriptor - offsets must match MJMetalVertex struct layout
        let vertexDescriptor = MTLVertexDescriptor()

        // Position
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0

        // Normal
        vertexDescriptor.attributes[1].format = .float3
        vertexDescriptor.attributes[1].offset = 16
        vertexDescriptor.attributes[1].bufferIndex = 0

        // TexCoord
        vertexDescriptor.attributes[2].format = .float2
        vertexDescriptor.attributes[2].offset = 32
        vertexDescriptor.attributes[2].bufferIndex = 0

        // Color (at offset 48, after padding)
        vertexDescriptor.attributes[3].format = .float4
        vertexDescriptor.attributes[3].offset = 48
        vertexDescriptor.attributes[3].bufferIndex = 0

        vertexDescriptor.layouts[0].stride = MemoryLayout<MJMetalVertex>.stride
        vertexDescriptor.layouts[0].stepFunction = .perVertex

        // Create pipeline
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.vertexFunction = vertexFunc
        pipelineDescriptor.fragmentFunction = fragmentFunc
        pipelineDescriptor.vertexDescriptor = vertexDescriptor
        pipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        pipelineDescriptor.depthAttachmentPixelFormat = .depth32Float

        // Enable blending
        pipelineDescriptor.colorAttachments[0].isBlendingEnabled = true
        pipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        pipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        pipelineDescriptor.colorAttachments[0].sourceAlphaBlendFactor = .one
        pipelineDescriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha

        self.pipelineState = try device.makeRenderPipelineState(descriptor: pipelineDescriptor)

        // Create depth state
        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .less
        depthDescriptor.isDepthWriteEnabled = true

        guard let depthState = device.makeDepthStencilState(descriptor: depthDescriptor) else {
            throw MJRendererError.depthStateCreationFailed
        }
        self.depthState = depthState

        // Allocate buffers
        guard let vBuffer = device.makeBuffer(length: maxVertices * MemoryLayout<MJMetalVertex>.stride,
                                               options: .storageModeShared),
              let iBuffer = device.makeBuffer(length: maxIndices * MemoryLayout<UInt32>.stride,
                                               options: .storageModeShared) else {
            throw MJRendererError.bufferAllocationFailed
        }
        self.vertexBuffer = vBuffer
        self.indexBuffer = iBuffer
    }

    // MARK: - Rendering (Lock-Free Ring Buffer API)

    public func render(frameData: UnsafePointer<MJFrameData>,
                       drawable: CAMetalDrawable,
                       renderPassDescriptor: MTLRenderPassDescriptor?) {

        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }

        let passDescriptor = renderPassDescriptor ?? MTLRenderPassDescriptor()
        passDescriptor.colorAttachments[0].texture = drawable.texture
        passDescriptor.colorAttachments[0].loadAction = .clear
        passDescriptor.colorAttachments[0].storeAction = .store
        // Background color - dark blue-gray
        passDescriptor.colorAttachments[0].clearColor = MTLClearColor(red: 0.15, green: 0.18, blue: 0.22, alpha: 1.0)

        // Setup depth texture
        let width = drawable.texture.width
        let height = drawable.texture.height

        if depthTexture == nil || depthTexture!.width != width || depthTexture!.height != height {
            let depthDesc = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: .depth32Float,
                width: width,
                height: height,
                mipmapped: false
            )
            depthDesc.usage = .renderTarget
            depthDesc.storageMode = .private
            depthTexture = device.makeTexture(descriptor: depthDesc)
        }

        passDescriptor.depthAttachment.texture = depthTexture
        passDescriptor.depthAttachment.loadAction = .clear
        passDescriptor.depthAttachment.storeAction = .dontCare
        passDescriptor.depthAttachment.clearDepth = 1.0

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: passDescriptor) else { return }

        encoder.setRenderPipelineState(pipelineState)
        encoder.setDepthStencilState(depthState)
        encoder.setCullMode(.none)

        // Setup camera matrices from frame data
        let aspect = Float(width) / Float(height)
        let fovy = Float(45.0 * .pi / 180.0)

        let frame = frameData.pointee
        let azRad = frame.camera_azimuth * .pi / 180.0
        let elRad = frame.camera_elevation * .pi / 180.0
        let dist = frame.camera_distance

        let eye = simd_float3(
            frame.camera_lookat.0 + dist * sin(azRad) * cos(elRad),
            frame.camera_lookat.1 + dist * cos(azRad) * cos(elRad),
            frame.camera_lookat.2 + dist * sin(elRad)
        )
        let center = simd_float3(frame.camera_lookat.0, frame.camera_lookat.1, frame.camera_lookat.2)
        let up = simd_float3(0, 0, 1)

        let viewMatrix = Self.lookAt(eye: eye, center: center, up: up)
        let projMatrix = Self.perspective(fovy: fovy, aspect: aspect, near: 0.01, far: 100.0)

        // Get buffer pointers
        let vertexData = vertexBuffer.contents().bindMemory(to: MJMetalVertex.self, capacity: maxVertices)
        let indexData = indexBuffer.contents().bindMemory(to: UInt32.self, capacity: maxIndices)

        var totalVertices = 0
        var totalIndices = 0

        // Get geom count through accessor function
        let geomCount = mjc_frame_get_geom_count(frameData)

        // Handle empty frame
        if geomCount == 0 {
            encoder.endEncoding()
            commandBuffer.present(drawable)
            commandBuffer.commit()
            return
        }

        // Maximum vertices/indices a single geometry can produce (conservative estimate)
        let maxVerticesPerGeom = 1000
        let maxIndicesPerGeom = 6000

        // Render each geometry from ring buffer frame data
        for i in 0..<Int(geomCount) {
            // Buffer overflow protection: skip if not enough space remains
            if totalVertices + maxVerticesPerGeom > maxVertices ||
               totalIndices + maxIndicesPerGeom > maxIndices {
                break
            }

            guard let geomPtr = mjc_frame_get_geom(frameData, Int32(i)) else { continue }
            let geom = geomPtr.pointee

            var vertexCount = 0
            var indexCount = 0

            Self.convertGeomInstance(
                geom: geom,
                vertices: vertexData.advanced(by: totalVertices),
                indices: indexData.advanced(by: totalIndices),
                vertexCount: &vertexCount,
                indexCount: &indexCount
            )

            if vertexCount > 0 && indexCount > 0 {
                // Build model matrix from geom pose
                var modelMatrix = simd_float4x4(1.0)
                modelMatrix.columns.0 = simd_float4(geom.mat.0, geom.mat.3, geom.mat.6, 0)
                modelMatrix.columns.1 = simd_float4(geom.mat.1, geom.mat.4, geom.mat.7, 0)
                modelMatrix.columns.2 = simd_float4(geom.mat.2, geom.mat.5, geom.mat.8, 0)
                modelMatrix.columns.3 = simd_float4(geom.pos.0, geom.pos.1, geom.pos.2, 1)

                // Get material properties from geom
                let emission = geom.emission
                let specular = geom.specular
                let shininess = min(max(geom.shininess, 1.0), 10.0)  // Clamp to reasonable range

                var uniforms = MJMetalUniforms(
                    modelMatrix: modelMatrix,
                    viewMatrix: viewMatrix,
                    projectionMatrix: projMatrix,
                    normalMatrix: Self.normalMatrix(from: modelMatrix),
                    lightPosition: simd_float3(0, 0, 10),
                    cameraPosition: eye,
                    color: simd_float4(1, 1, 1, 1),
                    emission: emission,
                    specular: specular,
                    shininess: shininess
                )

                encoder.setVertexBuffer(vertexBuffer, offset: totalVertices * MemoryLayout<MJMetalVertex>.stride, index: 0)
                encoder.setVertexBytes(&uniforms, length: MemoryLayout<MJMetalUniforms>.stride, index: 1)
                encoder.setFragmentBytes(&uniforms, length: MemoryLayout<MJMetalUniforms>.stride, index: 1)

                encoder.drawIndexedPrimitives(
                    type: .triangle,
                    indexCount: indexCount,
                    indexType: .uint32,
                    indexBuffer: indexBuffer,
                    indexBufferOffset: totalIndices * MemoryLayout<UInt32>.stride
                )

                totalVertices += vertexCount
                totalIndices += indexCount
            }
        }

        encoder.endEncoding()
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    // MARK: - Rendering (Legacy mjvScene API)

    public func render(scene: UnsafePointer<mjvScene>,
                       camera: UnsafePointer<mjvCamera>,
                       drawable: CAMetalDrawable,
                       renderPassDescriptor: MTLRenderPassDescriptor?) {

        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }

        let passDescriptor = renderPassDescriptor ?? MTLRenderPassDescriptor()
        passDescriptor.colorAttachments[0].texture = drawable.texture
        passDescriptor.colorAttachments[0].loadAction = .clear
        passDescriptor.colorAttachments[0].storeAction = .store
        passDescriptor.colorAttachments[0].clearColor = MTLClearColor(red: 0.15, green: 0.18, blue: 0.22, alpha: 1.0)

        // Setup depth texture
        let width = drawable.texture.width
        let height = drawable.texture.height

        if depthTexture == nil || depthTexture!.width != width || depthTexture!.height != height {
            let depthDesc = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: .depth32Float,
                width: width,
                height: height,
                mipmapped: false
            )
            depthDesc.usage = .renderTarget
            depthDesc.storageMode = .private
            depthTexture = device.makeTexture(descriptor: depthDesc)
        }

        passDescriptor.depthAttachment.texture = depthTexture
        passDescriptor.depthAttachment.loadAction = .clear
        passDescriptor.depthAttachment.storeAction = .dontCare
        passDescriptor.depthAttachment.clearDepth = 1.0

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: passDescriptor) else { return }

        encoder.setRenderPipelineState(pipelineState)
        encoder.setDepthStencilState(depthState)
        encoder.setCullMode(.none)

        // Setup camera matrices
        let aspect = Float(width) / Float(height)
        let fovy = Float(45.0 * .pi / 180.0)

        let cam = camera.pointee
        let azRad = Float(cam.azimuth) * .pi / 180.0
        let elRad = Float(cam.elevation) * .pi / 180.0
        let dist = Float(cam.distance)

        let eye = simd_float3(
            Float(cam.lookat.0) + dist * sin(azRad) * cos(elRad),
            Float(cam.lookat.1) + dist * cos(azRad) * cos(elRad),
            Float(cam.lookat.2) + dist * sin(elRad)
        )
        let center = simd_float3(Float(cam.lookat.0), Float(cam.lookat.1), Float(cam.lookat.2))
        let up = simd_float3(0, 0, 1)

        let viewMatrix = Self.lookAt(eye: eye, center: center, up: up)
        let projMatrix = Self.perspective(fovy: fovy, aspect: aspect, near: 0.01, far: 100.0)

        // Get buffer pointers
        let vertexData = vertexBuffer.contents().bindMemory(to: MJMetalVertex.self, capacity: maxVertices)
        let indexData = indexBuffer.contents().bindMemory(to: UInt32.self, capacity: maxIndices)

        var totalVertices = 0
        var totalIndices = 0

        // Render each geometry
        let scn = scene.pointee

        // Handle empty scene
        if scn.ngeom == 0 || scn.geoms == nil {
            encoder.endEncoding()
            commandBuffer.present(drawable)
            commandBuffer.commit()
            return
        }

        let geomsPtr = scn.geoms!

        // Maximum vertices/indices a single geometry can produce (conservative estimate)
        let maxVerticesPerGeom = 1000
        let maxIndicesPerGeom = 6000

        for i in 0..<Int(scn.ngeom) {
            // Buffer overflow protection: skip if not enough space remains
            if totalVertices + maxVerticesPerGeom > maxVertices ||
               totalIndices + maxIndicesPerGeom > maxIndices {
                break
            }

            let geom = geomsPtr[i]

            var vertexCount = 0
            var indexCount = 0

            Self.convertGeom(
                geom: geom,
                vertices: vertexData.advanced(by: totalVertices),
                indices: indexData.advanced(by: totalIndices),
                vertexCount: &vertexCount,
                indexCount: &indexCount
            )

            if vertexCount > 0 && indexCount > 0 {
                // Build model matrix from geom pose
                var modelMatrix = simd_float4x4(1.0)
                modelMatrix.columns.0 = simd_float4(geom.mat.0, geom.mat.3, geom.mat.6, 0)
                modelMatrix.columns.1 = simd_float4(geom.mat.1, geom.mat.4, geom.mat.7, 0)
                modelMatrix.columns.2 = simd_float4(geom.mat.2, geom.mat.5, geom.mat.8, 0)
                modelMatrix.columns.3 = simd_float4(geom.pos.0, geom.pos.1, geom.pos.2, 1)

                let emission = geom.emission
                let specular = geom.specular
                let shininess = min(max(geom.shininess, 1.0), 10.0)  // Clamp to reasonable range

                var uniforms = MJMetalUniforms(
                    modelMatrix: modelMatrix,
                    viewMatrix: viewMatrix,
                    projectionMatrix: projMatrix,
                    normalMatrix: Self.normalMatrix(from: modelMatrix),
                    lightPosition: simd_float3(0, 0, 10),
                    cameraPosition: eye,
                    color: simd_float4(1, 1, 1, 1),
                    emission: emission,
                    specular: specular,
                    shininess: shininess
                )

                encoder.setVertexBuffer(vertexBuffer, offset: totalVertices * MemoryLayout<MJMetalVertex>.stride, index: 0)
                encoder.setVertexBytes(&uniforms, length: MemoryLayout<MJMetalUniforms>.stride, index: 1)
                encoder.setFragmentBytes(&uniforms, length: MemoryLayout<MJMetalUniforms>.stride, index: 1)

                encoder.drawIndexedPrimitives(
                    type: .triangle,
                    indexCount: indexCount,
                    indexType: .uint32,
                    indexBuffer: indexBuffer,
                    indexBufferOffset: totalIndices * MemoryLayout<UInt32>.stride
                )

                totalVertices += vertexCount
                totalIndices += indexCount
            }
        }

        encoder.endEncoding()
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    // MARK: - Matrix Helpers

    private static func perspective(fovy: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let ys = 1.0 / tan(fovy * 0.5)
        let xs = ys / aspect
        let zs = far / (near - far)

        return simd_float4x4(columns: (
            simd_float4(xs, 0, 0, 0),
            simd_float4(0, ys, 0, 0),
            simd_float4(0, 0, zs, -1),
            simd_float4(0, 0, near * zs, 0)
        ))
    }

    private static func lookAt(eye: simd_float3, center: simd_float3, up: simd_float3) -> simd_float4x4 {
        let f = simd_normalize(center - eye)
        let s = simd_normalize(simd_cross(f, up))
        let u = simd_cross(s, f)

        return simd_float4x4(columns: (
            simd_float4(s.x, u.x, -f.x, 0),
            simd_float4(s.y, u.y, -f.y, 0),
            simd_float4(s.z, u.z, -f.z, 0),
            simd_float4(-simd_dot(s, eye), -simd_dot(u, eye), simd_dot(f, eye), 1)
        ))
    }

    /// Compute normal matrix (inverse transpose of upper-left 3x3) for correct lighting
    /// with non-uniform scaling. Returns identity-based 4x4 matrix with 3x3 normal transform.
    private static func normalMatrix(from modelMatrix: simd_float4x4) -> simd_float4x4 {
        // Extract upper-left 3x3
        let m = simd_float3x3(
            simd_float3(modelMatrix.columns.0.x, modelMatrix.columns.0.y, modelMatrix.columns.0.z),
            simd_float3(modelMatrix.columns.1.x, modelMatrix.columns.1.y, modelMatrix.columns.1.z),
            simd_float3(modelMatrix.columns.2.x, modelMatrix.columns.2.y, modelMatrix.columns.2.z)
        )

        // Compute inverse transpose (handles non-uniform scaling correctly)
        let invTranspose = m.inverse.transpose

        // Pack back into 4x4 matrix
        return simd_float4x4(columns: (
            simd_float4(invTranspose.columns.0.x, invTranspose.columns.0.y, invTranspose.columns.0.z, 0),
            simd_float4(invTranspose.columns.1.x, invTranspose.columns.1.y, invTranspose.columns.1.z, 0),
            simd_float4(invTranspose.columns.2.x, invTranspose.columns.2.y, invTranspose.columns.2.z, 0),
            simd_float4(0, 0, 0, 1)
        ))
    }

    // MARK: - Geometry Conversion

    /// Convert MJGeomInstance (from ring buffer) to mesh vertices
    private static func convertGeomInstance(geom: MJGeomInstance,
                                            vertices: UnsafeMutablePointer<MJMetalVertex>,
                                            indices: UnsafeMutablePointer<UInt32>,
                                            vertexCount: inout Int,
                                            indexCount: inout Int) {
        let color = simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3)
        let size = (geom.size.0, geom.size.1, geom.size.2)

        switch geom.type {
        case 0: // mjGEOM_PLANE
            generatePlane(size: size, color: color, vertices: vertices, indices: indices,
                         vertexCount: &vertexCount, indexCount: &indexCount)

        case 2: // mjGEOM_SPHERE
            generateSphere(radius: geom.size.0, color: color, vertices: vertices, indices: indices,
                          vertexCount: &vertexCount, indexCount: &indexCount)

        case 3: // mjGEOM_CAPSULE
            generateCapsule(radius: geom.size.0, halfLength: geom.size.2, color: color,
                           vertices: vertices, indices: indices,
                           vertexCount: &vertexCount, indexCount: &indexCount)

        case 5: // mjGEOM_CYLINDER
            generateCylinder(radius: geom.size.0, halfLength: geom.size.2, color: color,
                            vertices: vertices, indices: indices,
                            vertexCount: &vertexCount, indexCount: &indexCount)

        case 6: // mjGEOM_BOX
            generateBox(size: size, color: color, vertices: vertices, indices: indices,
                       vertexCount: &vertexCount, indexCount: &indexCount)

        default:
            // Placeholder cube for unsupported types
            generateBox(size: (0.02, 0.02, 0.02), color: color, vertices: vertices, indices: indices,
                       vertexCount: &vertexCount, indexCount: &indexCount)
        }
    }

    /// Convert mjvGeom (legacy API) to mesh vertices
    private static func convertGeom(geom: mjvGeom,
                                    vertices: UnsafeMutablePointer<MJMetalVertex>,
                                    indices: UnsafeMutablePointer<UInt32>,
                                    vertexCount: inout Int,
                                    indexCount: inout Int) {
        let color = simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3)

        switch geom.type {
        case 0: // mjGEOM_PLANE
            generatePlane(size: geom.size, color: color, vertices: vertices, indices: indices,
                         vertexCount: &vertexCount, indexCount: &indexCount)

        case 2: // mjGEOM_SPHERE
            generateSphere(radius: geom.size.0, color: color, vertices: vertices, indices: indices,
                          vertexCount: &vertexCount, indexCount: &indexCount)

        case 3: // mjGEOM_CAPSULE
            generateCapsule(radius: geom.size.0, halfLength: geom.size.2, color: color,
                           vertices: vertices, indices: indices,
                           vertexCount: &vertexCount, indexCount: &indexCount)

        case 5: // mjGEOM_CYLINDER
            generateCylinder(radius: geom.size.0, halfLength: geom.size.2, color: color,
                            vertices: vertices, indices: indices,
                            vertexCount: &vertexCount, indexCount: &indexCount)

        case 6: // mjGEOM_BOX
            generateBox(size: geom.size, color: color, vertices: vertices, indices: indices,
                       vertexCount: &vertexCount, indexCount: &indexCount)

        default:
            // Placeholder cube for unsupported types
            generateBox(size: (0.02, 0.02, 0.02), color: color, vertices: vertices, indices: indices,
                       vertexCount: &vertexCount, indexCount: &indexCount)
        }
    }

    // MARK: - Geometry Generators

    private static func generatePlane(size: (Float, Float, Float), color: simd_float4,
                                      vertices: UnsafeMutablePointer<MJMetalVertex>,
                                      indices: UnsafeMutablePointer<UInt32>,
                                      vertexCount: inout Int, indexCount: inout Int) {
        let sx = size.0 > 0 ? size.0 : 10.0
        let sy = size.1 > 0 ? size.1 : 10.0
        let normal = simd_float3(0, 0, 1)

        vertices[0] = MJMetalVertex(position: simd_float3(-sx, -sy, 0), normal: normal, texCoord: simd_float2(0, 0), color: color)
        vertices[1] = MJMetalVertex(position: simd_float3(sx, -sy, 0), normal: normal, texCoord: simd_float2(1, 0), color: color)
        vertices[2] = MJMetalVertex(position: simd_float3(sx, sy, 0), normal: normal, texCoord: simd_float2(1, 1), color: color)
        vertices[3] = MJMetalVertex(position: simd_float3(-sx, sy, 0), normal: normal, texCoord: simd_float2(0, 1), color: color)

        indices[0] = 0; indices[1] = 1; indices[2] = 2
        indices[3] = 0; indices[4] = 2; indices[5] = 3

        vertexCount = 4
        indexCount = 6
    }

    private static func generateSphere(radius: Float, color: simd_float4,
                                       vertices: UnsafeMutablePointer<MJMetalVertex>,
                                       indices: UnsafeMutablePointer<UInt32>,
                                       vertexCount: inout Int, indexCount: inout Int) {
        let segments = 16
        let rings = 12
        var vCount = 0
        var iCount = 0

        for r in 0...rings {
            let phi = Float.pi * Float(r) / Float(rings)
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)

                let x = radius * sin(phi) * cos(theta)
                let y = radius * sin(phi) * sin(theta)
                let z = radius * cos(phi)

                let pos = simd_float3(x, y, z)
                let normal = simd_normalize(pos)
                let texCoord = simd_float2(Float(s) / Float(segments), Float(r) / Float(rings))

                vertices[vCount] = MJMetalVertex(position: pos, normal: normal, texCoord: texCoord, color: color)
                vCount += 1
            }
        }

        for r in 0..<rings {
            for s in 0..<segments {
                let curr = r * (segments + 1) + s
                let next = curr + segments + 1

                indices[iCount] = UInt32(curr); iCount += 1
                indices[iCount] = UInt32(next); iCount += 1
                indices[iCount] = UInt32(curr + 1); iCount += 1

                indices[iCount] = UInt32(curr + 1); iCount += 1
                indices[iCount] = UInt32(next); iCount += 1
                indices[iCount] = UInt32(next + 1); iCount += 1
            }
        }

        vertexCount = vCount
        indexCount = iCount
    }

    private static func generateBox(size: (Float, Float, Float), color: simd_float4,
                                    vertices: UnsafeMutablePointer<MJMetalVertex>,
                                    indices: UnsafeMutablePointer<UInt32>,
                                    vertexCount: inout Int, indexCount: inout Int) {
        let sx = size.0, sy = size.1, sz = size.2

        let corners: [simd_float3] = [
            simd_float3(-sx, -sy, -sz), simd_float3(sx, -sy, -sz),
            simd_float3(sx, sy, -sz), simd_float3(-sx, sy, -sz),
            simd_float3(-sx, -sy, sz), simd_float3(sx, -sy, sz),
            simd_float3(sx, sy, sz), simd_float3(-sx, sy, sz)
        ]

        let faceIndices: [[Int]] = [
            [0, 1, 2, 3], [4, 7, 6, 5],
            [0, 4, 5, 1], [2, 6, 7, 3],
            [0, 3, 7, 4], [1, 5, 6, 2]
        ]
        let faceNormals: [simd_float3] = [
            simd_float3(0, 0, -1), simd_float3(0, 0, 1),
            simd_float3(0, -1, 0), simd_float3(0, 1, 0),
            simd_float3(-1, 0, 0), simd_float3(1, 0, 0)
        ]

        var vCount = 0
        var iCount = 0

        for f in 0..<6 {
            let baseVertex = vCount
            for v in 0..<4 {
                vertices[vCount] = MJMetalVertex(
                    position: corners[faceIndices[f][v]],
                    normal: faceNormals[f],
                    texCoord: simd_float2(Float(v % 2), Float(v / 2)),
                    color: color
                )
                vCount += 1
            }
            indices[iCount] = UInt32(baseVertex); iCount += 1
            indices[iCount] = UInt32(baseVertex + 1); iCount += 1
            indices[iCount] = UInt32(baseVertex + 2); iCount += 1
            indices[iCount] = UInt32(baseVertex); iCount += 1
            indices[iCount] = UInt32(baseVertex + 2); iCount += 1
            indices[iCount] = UInt32(baseVertex + 3); iCount += 1
        }

        vertexCount = vCount
        indexCount = iCount
    }

    private static func generateCylinder(radius: Float, halfLength: Float, color: simd_float4,
                                         vertices: UnsafeMutablePointer<MJMetalVertex>,
                                         indices: UnsafeMutablePointer<UInt32>,
                                         vertexCount: inout Int, indexCount: inout Int) {
        let segments = 16
        var vCount = 0
        var iCount = 0

        for s in 0...segments {
            let theta = 2.0 * Float.pi * Float(s) / Float(segments)
            let x = radius * cos(theta)
            let y = radius * sin(theta)
            let normal = simd_float3(cos(theta), sin(theta), 0)

            // Bottom vertex
            vertices[vCount] = MJMetalVertex(
                position: simd_float3(x, y, -halfLength),
                normal: normal,
                texCoord: simd_float2(Float(s) / Float(segments), 0),
                color: color
            )
            vCount += 1

            // Top vertex
            vertices[vCount] = MJMetalVertex(
                position: simd_float3(x, y, halfLength),
                normal: normal,
                texCoord: simd_float2(Float(s) / Float(segments), 1),
                color: color
            )
            vCount += 1
        }

        for s in 0..<segments {
            let curr = s * 2
            indices[iCount] = UInt32(curr); iCount += 1
            indices[iCount] = UInt32(curr + 2); iCount += 1
            indices[iCount] = UInt32(curr + 1); iCount += 1
            indices[iCount] = UInt32(curr + 1); iCount += 1
            indices[iCount] = UInt32(curr + 2); iCount += 1
            indices[iCount] = UInt32(curr + 3); iCount += 1
        }

        vertexCount = vCount
        indexCount = iCount
    }

    private static func generateCapsule(radius: Float, halfLength: Float, color: simd_float4,
                                        vertices: UnsafeMutablePointer<MJMetalVertex>,
                                        indices: UnsafeMutablePointer<UInt32>,
                                        vertexCount: inout Int, indexCount: inout Int) {
        // Start with cylinder
        generateCylinder(radius: radius, halfLength: halfLength, color: color,
                        vertices: vertices, indices: indices,
                        vertexCount: &vertexCount, indexCount: &indexCount)

        // Add hemisphere caps
        let segments = 16
        let rings = 8

        // Bottom hemisphere
        var capBaseVertex = vertexCount
        for r in 0...rings {
            let phi = Float.pi * 0.5 + Float.pi * 0.5 * Float(r) / Float(rings)
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = radius * sin(phi) * cos(theta)
                let y = radius * sin(phi) * sin(theta)
                let z = radius * cos(phi) - halfLength

                let pos = simd_float3(x, y, z)
                let normal = simd_normalize(simd_float3(x, y, z + halfLength))

                vertices[vertexCount] = MJMetalVertex(
                    position: pos, normal: normal,
                    texCoord: simd_float2(Float(s) / Float(segments), Float(r) / Float(rings)),
                    color: color
                )
                vertexCount += 1
            }
        }

        for r in 0..<rings {
            for s in 0..<segments {
                let curr = capBaseVertex + r * (segments + 1) + s
                let next = curr + segments + 1
                indices[indexCount] = UInt32(curr); indexCount += 1
                indices[indexCount] = UInt32(next); indexCount += 1
                indices[indexCount] = UInt32(curr + 1); indexCount += 1
                indices[indexCount] = UInt32(curr + 1); indexCount += 1
                indices[indexCount] = UInt32(next); indexCount += 1
                indices[indexCount] = UInt32(next + 1); indexCount += 1
            }
        }

        // Top hemisphere
        capBaseVertex = vertexCount
        for r in 0...rings {
            let phi = Float.pi * 0.5 * Float(r) / Float(rings)
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = radius * sin(Float.pi * 0.5 - phi) * cos(theta)
                let y = radius * sin(Float.pi * 0.5 - phi) * sin(theta)
                let z = radius * cos(Float.pi * 0.5 - phi) + halfLength

                let pos = simd_float3(x, y, z)
                let normal = simd_normalize(simd_float3(x, y, z - halfLength))

                vertices[vertexCount] = MJMetalVertex(
                    position: pos, normal: normal,
                    texCoord: simd_float2(Float(s) / Float(segments), Float(r) / Float(rings)),
                    color: color
                )
                vertexCount += 1
            }
        }

        for r in 0..<rings {
            for s in 0..<segments {
                let curr = capBaseVertex + r * (segments + 1) + s
                let next = curr + segments + 1
                indices[indexCount] = UInt32(curr); indexCount += 1
                indices[indexCount] = UInt32(curr + 1); indexCount += 1
                indices[indexCount] = UInt32(next); indexCount += 1
                indices[indexCount] = UInt32(curr + 1); indexCount += 1
                indices[indexCount] = UInt32(next + 1); indexCount += 1
                indices[indexCount] = UInt32(next); indexCount += 1
            }
        }
    }
}

// MARK: - Errors

public enum MJRendererError: Error {
    case commandQueueCreationFailed
    case shaderCompilationFailed
    case shaderFunctionNotFound
    case pipelineCreationFailed
    case depthStateCreationFailed
    case bufferAllocationFailed
}
