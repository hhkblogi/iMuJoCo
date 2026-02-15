// mjc_metal_render.swift
// Pure Swift Metal render for MuJoCo scenes

import Metal
import MetalKit
import simd
import os.log
import MJCPhysicsRuntime

private let logger = Logger(subsystem: "com.mujoco.render", category: "MJCMetalRender")

// MARK: - Shader Types

struct MJCMetalUniforms {
    var modelMatrix: simd_float4x4
    var viewMatrix: simd_float4x4
    var projectionMatrix: simd_float4x4
    var normal_matrix: simd_float4x4
    var lightPosition: simd_float3
    var _padding0: Float  // Alignment padding after lightPosition (float3 aligns to 16 bytes in Metal)
    var cameraPosition: simd_float3
    var _padding1: Float  // Alignment padding after cameraPosition
    var color: simd_float4
    var emission: Float
    var specular: Float
    var shininess: Float
    var checkerboardScale: Float  // >0 = procedural checkerboard cell size, 0 = disabled
}

struct MJCMetalVertex {
    var position: simd_float3      // offset 0, size 12, padded to 16
    var normal: simd_float3        // offset 16, size 12, padded to 16
    var texCoord: simd_float2      // offset 32, size 8
    var _padding: simd_float2      // offset 40, size 8 - pad to 16-byte boundary
    var color: simd_float4         // offset 48, size 16
    // Total: 64 bytes

    /// Convenience initializer that defaults padding to zero.
    init(position: simd_float3, normal: simd_float3, texCoord: simd_float2, color: simd_float4) {
        self.position = position
        self.normal = normal
        self.texCoord = texCoord
        self._padding = .zero
        self.color = color
    }
}

// MARK: - Light Types (must match Metal Light/LightBuffer structs)

struct MJCMetalLight {
    var pos: simd_float3; var _pad0: Float
    var dir: simd_float3; var _pad1: Float
    var ambient: simd_float3; var _pad2: Float
    var diffuse: simd_float3; var _pad3: Float
    var specular: simd_float3; var _pad4: Float
    var attenuation: simd_float3; var cutoff: Float
    var exponent: Float
    var headlight: Int32
    var directional: Int32
    var _pad5: Float
}

struct MJCLightBuffer {
    var lightCount: Int32
    var _pad: (Int32, Int32, Int32)
    var lights: (MJCMetalLight, MJCMetalLight, MJCMetalLight, MJCMetalLight,
                 MJCMetalLight, MJCMetalLight, MJCMetalLight, MJCMetalLight)
}

// MARK: - Metal Render

/// Metal-based render for MuJoCo physics visualization.
///
/// `MJCMetalRender` converts MuJoCo geometry data into Metal vertex buffers and renders
/// using custom shaders with Blinn-Phong lighting. It supports both the lock-free
/// `MJFrameData` ring buffer API and legacy `mjvScene` for backwards compatibility.
///
/// ## Usage
/// ```swift
/// let render = try MJCMetalRender(device: MTLCreateSystemDefaultDevice()!)
/// render.Render(frameData: frame, drawable: drawable, renderPassDescriptor: descriptor)
/// ```
///
/// ## Thread Safety
/// The render is designed for single-threaded use from the main/render thread.
/// Frame data can be produced on a separate physics thread using the ring buffer API.
public final class MJCMetalRender {
    private let device: MTLDevice
    private let command_queue: MTLCommandQueue
    private let pipeline_state: MTLRenderPipelineState
    private let depth_state: MTLDepthStencilState
    private let gizmoDepthState: MTLDepthStencilState
    private let gizmoVertexBuffer: MTLBuffer
    private let gizmoIndexBuffer: MTLBuffer
    private let gizmoIndexCounts: [(offset: Int, count: Int)]  // per-axis (X, Y, Z) index ranges

    private var depth_texture: MTLTexture?

    // MARK: - Mesh Cache (static geometry)

    /// Cached immutable Metal buffers for a single mesh (created once at load time).
    private struct CachedMesh {
        let vertexBuffer: MTLBuffer
        let indexBuffer: MTLBuffer
        let vertexCount: Int
        let indexCount: Int
    }
    /// meshId → cached MTLBuffers (populated lazily on first frame with mesh data)
    private var meshCache: [Int: CachedMesh] = [:]
    private var meshCacheBuilt = false

    // MARK: - Triple Buffering (dynamic primitive geometry)

    private static let inflightFrameCount = 3
    private var dynamicVertexBuffers: [MTLBuffer] = []
    private var dynamicIndexBuffers: [MTLBuffer] = []
    private var bufferIndex = 0
    private let inflightSemaphore = DispatchSemaphore(value: inflightFrameCount)

    // Dynamic buffer capacity - sized for procedural primitives only (meshes use cached buffers).
    // Reduced from 2M because dynamic buffers no longer hold mesh vertices.
    private let max_vertices = 512 * 1024  // ~32MB per buffer at 64 bytes/vertex
    private let max_indices = 512 * 1024   // ~2MB per buffer at 4 bytes/index

    // MARK: - Brightness Readback (GPU → CPU)

    /// Triple-buffered readback buffers for sampling rendered pixel brightness.
    /// Each buffer holds a 4×4 BGRA8 pixel patch (64 bytes).
    private var brightnessReadbackBuffers: [MTLBuffer] = []
    private var brightnessReadbackValid: [Bool] = [false, false, false]

    /// Average luminance of the rendered scene (0.0 dark – 1.0 bright).
    /// Updated each frame from the previous frame's GPU readback (1-frame latency, no stall).
    public private(set) var renderedBrightness: Float = 0.0

    /// Upper bounds on vertices/indices a single procedural geometry can contribute to a frame.
    /// Used as a buffer capacity guard so we can detect when adding a geom would exceed
    /// the pre-allocated vertex/index buffers and skip it instead of writing past buffer bounds.
    ///
    /// The values are conservative and sized to cover the most tessellated supported MuJoCo
    /// geom types (spheres, cylinders, capsules, ellipsoids at 16 segments × 12 rings = ~192 quads
    /// = ~384 triangles = ~1152 indices per curved surface) with additional headroom.
    /// Simpler geoms (boxes, planes, segments) use substantially fewer vertices and indices.
    private static let max_vertices_per_geom = 1000
    private static let max_indices_per_geom = 6000

    // MARK: - Initialization

    /// Creates a new Metal render with the specified device.
    ///
    /// - Parameter device: The Metal device to use for rendering.
    /// - Throws: `MJCRenderError` if initialization fails (command queue, shaders, or buffers).
    public init(device: MTLDevice) throws {
        self.device = device

        guard let queue = device.makeCommandQueue() else {
            throw MJCRenderError.command_queueCreationFailed
        }
        self.command_queue = queue

        // Load shaders from compiled Metal library in the framework bundle
        let bundle = Bundle(for: MJCMetalRender.self)
        guard let library = try? device.makeDefaultLibrary(bundle: bundle) else {
            throw MJCRenderError.shaderCompilationFailed
        }

        guard let vertexFunc = library.makeFunction(name: "vertexMain"),
              let fragmentFunc = library.makeFunction(name: "fragmentMain") else {
            throw MJCRenderError.shaderFunctionNotFound
        }

        // Create vertex descriptor - offsets must match MJCMetalVertex struct layout
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

        vertexDescriptor.layouts[0].stride = MemoryLayout<MJCMetalVertex>.stride
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

        self.pipeline_state = try device.makeRenderPipelineState(descriptor: pipelineDescriptor)

        // Create depth state
        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .less
        depthDescriptor.isDepthWriteEnabled = true

        guard let depth_state = device.makeDepthStencilState(descriptor: depthDescriptor) else {
            throw MJCRenderError.depth_stateCreationFailed
        }
        self.depth_state = depth_state

        // Create gizmo depth state (always pass, no depth write — painter's algorithm)
        let gizmoDepthDesc = MTLDepthStencilDescriptor()
        gizmoDepthDesc.depthCompareFunction = .always
        gizmoDepthDesc.isDepthWriteEnabled = false
        guard let gizmoDS = device.makeDepthStencilState(descriptor: gizmoDepthDesc) else {
            throw MJCRenderError.depth_stateCreationFailed
        }
        self.gizmoDepthState = gizmoDS

        // Build gizmo geometry (3 axis arrows in a single buffer)
        guard let gizmo = Self.buildGizmoGeometry(device: device) else {
            throw MJCRenderError.bufferAllocationFailed
        }
        self.gizmoVertexBuffer = gizmo.vertexBuffer
        self.gizmoIndexBuffer = gizmo.indexBuffer
        self.gizmoIndexCounts = gizmo.indexCounts

        // Allocate triple-buffered dynamic buffers for procedural geometry
        for _ in 0..<Self.inflightFrameCount {
            guard let vBuf = device.makeBuffer(length: max_vertices * MemoryLayout<MJCMetalVertex>.stride,
                                                options: .storageModeShared),
                  let iBuf = device.makeBuffer(length: max_indices * MemoryLayout<UInt32>.stride,
                                                options: .storageModeShared) else {
                throw MJCRenderError.bufferAllocationFailed
            }
            dynamicVertexBuffers.append(vBuf)
            dynamicIndexBuffers.append(iBuf)
        }

        // Allocate triple-buffered readback buffers for brightness sampling (4×4 BGRA8 = 64 bytes each)
        for _ in 0..<Self.inflightFrameCount {
            guard let buf = device.makeBuffer(length: 4 * 4 * 4, options: .storageModeShared) else {
                throw MJCRenderError.bufferAllocationFailed
            }
            brightnessReadbackBuffers.append(buf)
        }
    }

    // MARK: - Mesh Cache Builder

    /// Build immutable Metal buffers for all meshes. Called once when meshData first becomes available.
    private func buildMeshCache(meshData: MJMeshData) {
        guard !meshCacheBuilt else { return }

        guard let meshInfoPtr = MJMeshDataGetMeshes(meshData),
              let meshVertPtr = MJMeshDataGetVertices(meshData),
              let meshFacePtr = MJMeshDataGetFaces(meshData) else {
            return
        }
        meshCacheBuilt = true

        let meshCount = Int(meshData.meshCount())
        for meshId in 0..<meshCount {
            let info = meshInfoPtr[meshId]
            let nv = Int(info.vertexCount)
            let nf = Int(info.faceCount)
            guard nv > 0, nf > 0 else { continue }

            // Build vertex data with white color (actual color applied via uniforms.color)
            let vertexStride = MemoryLayout<MJCMetalVertex>.stride
            guard let vBuf = device.makeBuffer(length: nv * vertexStride, options: .storageModeShared) else {
                logger.warning("Failed to allocate vertex buffer for mesh \(meshId)")
                continue
            }
            let vPtr = vBuf.contents().bindMemory(to: MJCMetalVertex.self, capacity: nv)
            let vertBase = Int(info.vertexOffset) * 6
            let whiteColor = simd_float4(1, 1, 1, 1)
            for v in 0..<nv {
                let srcIdx = vertBase + v * 6
                let pos = simd_float3(meshVertPtr[srcIdx], meshVertPtr[srcIdx + 1], meshVertPtr[srcIdx + 2])
                let normal = simd_float3(meshVertPtr[srcIdx + 3], meshVertPtr[srcIdx + 4], meshVertPtr[srcIdx + 5])
                vPtr[v] = MJCMetalVertex(position: pos, normal: normal, texCoord: .zero, color: whiteColor)
            }

            // Build index data
            let indexCount = nf * 3
            guard let iBuf = device.makeBuffer(length: indexCount * MemoryLayout<UInt32>.stride,
                                                options: .storageModeShared) else {
                logger.warning("Failed to allocate index buffer for mesh \(meshId)")
                continue
            }
            let iPtr = iBuf.contents().bindMemory(to: UInt32.self, capacity: indexCount)
            let faceBase = Int(info.faceOffset) * 3
            for f in 0..<indexCount {
                iPtr[f] = UInt32(meshFacePtr[faceBase + f])
            }

            meshCache[meshId] = CachedMesh(vertexBuffer: vBuf, indexBuffer: iBuf,
                                            vertexCount: nv, indexCount: indexCount)
        }

        logger.info("Built mesh cache: \(self.meshCache.count) meshes cached")
    }

    // MARK: - Rendering (Lock-Free Ring Buffer API)

    /// Renders a frame using the lock-free ring buffer API (preferred).
    ///
    /// This method renders geometry from an `MJFrameData` structure, which is typically
    /// obtained from the physics runtime's ring buffer. This is the recommended rendering
    /// path for optimal performance and thread safety.
    ///
    /// - Parameters:
    ///   - frame: The frame data view (SWIFT_IMMORTAL_REFERENCE class).
    ///   - drawable: The Metal drawable to render into.
    ///   - renderPassDescriptor: Optional render pass descriptor. If nil, a default is created.
    public func Render(frame: MJFrameData,
                       meshData: MJMeshData? = nil,
                       drawable: CAMetalDrawable,
                       renderPassDescriptor: MTLRenderPassDescriptor?) {

        // MJFrameData is exposed to Swift as a reference type (SWIFT_IMMORTAL_REFERENCE).
        // IMPORTANT: The underlying storage is managed via thread-local storage and is only
        // valid until the next getLatestFrame() call on the same thread. Do NOT store the
        // frame reference beyond this render call.

        // Build mesh cache lazily on first frame with mesh data
        if !meshCacheBuilt, let md = meshData {
            buildMeshCache(meshData: md)
        }

        // Triple buffering: wait for a free buffer slot
        inflightSemaphore.wait()
        let currentBufferIndex = bufferIndex
        bufferIndex = (bufferIndex + 1) % Self.inflightFrameCount
        let dynamicVB = dynamicVertexBuffers[currentBufferIndex]
        let dynamicIB = dynamicIndexBuffers[currentBufferIndex]

        // Read brightness from this slot's previous blit (guaranteed complete after semaphore wait)
        if brightnessReadbackValid[currentBufferIndex] {
            let buf = brightnessReadbackBuffers[currentBufferIndex]
            let ptr = buf.contents().bindMemory(to: UInt8.self, capacity: 64)
            var totalLum: Float = 0
            for i in 0..<16 {  // 4×4 pixels
                let offset = i * 4  // BGRA8
                let b = Float(ptr[offset]) / 255.0
                let g = Float(ptr[offset + 1]) / 255.0
                let r = Float(ptr[offset + 2]) / 255.0
                totalLum += 0.299 * r + 0.587 * g + 0.114 * b
            }
            renderedBrightness = totalLum / 16.0
        }

        guard let commandBuffer = command_queue.makeCommandBuffer() else {
            inflightSemaphore.signal()
            return
        }
        commandBuffer.addCompletedHandler { [sem = self.inflightSemaphore] _ in sem.signal() }

        let passDescriptor = renderPassDescriptor ?? MTLRenderPassDescriptor()
        passDescriptor.colorAttachments[0].texture = drawable.texture
        passDescriptor.colorAttachments[0].loadAction = .clear
        passDescriptor.colorAttachments[0].storeAction = .store
        // Background color - dark blue-gray
        passDescriptor.colorAttachments[0].clearColor = MTLClearColor(red: 0.15, green: 0.18, blue: 0.22, alpha: 1.0)

        // Setup depth texture
        let width = drawable.texture.width
        let height = drawable.texture.height

        if depth_texture == nil || depth_texture!.width != width || depth_texture!.height != height {
            let depthDesc = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: .depth32Float,
                width: width,
                height: height,
                mipmapped: false
            )
            depthDesc.usage = .renderTarget
            depthDesc.storageMode = .private
            depth_texture = device.makeTexture(descriptor: depthDesc)
        }

        passDescriptor.depthAttachment.texture = depth_texture
        passDescriptor.depthAttachment.loadAction = .clear
        passDescriptor.depthAttachment.storeAction = .dontCare
        passDescriptor.depthAttachment.clearDepth = 1.0

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: passDescriptor) else {
            commandBuffer.commit()
            return
        }

        encoder.setRenderPipelineState(pipeline_state)
        encoder.setDepthStencilState(depth_state)
        encoder.setCullMode(.none)

        // Setup camera matrices from frame data
        let aspect = Float(width) / Float(height)
        let fovy = Float(45.0 * .pi / 180.0)

        let azRad = frame.cameraAzimuth() * .pi / 180.0
        let elRad = frame.cameraElevation() * .pi / 180.0
        let dist = frame.cameraDistance()

        // Access camera lookat
        let lookatX = frame.cameraLookatX()
        let lookatY = frame.cameraLookatY()
        let lookatZ = frame.cameraLookatZ()

        let eye = simd_float3(
            lookatX + dist * sin(azRad) * cos(elRad),
            lookatY + dist * cos(azRad) * cos(elRad),
            lookatZ + dist * sin(elRad)
        )
        let center = simd_float3(lookatX, lookatY, lookatZ)
        let up = simd_float3(0, 0, 1)

        let viewMatrix = Self.look_at(eye: eye, center: center, up: up)
        let projMatrix = Self.perspective(fovy: fovy, aspect: aspect, near: 0.01, far: 500.0)

        // Get dynamic buffer pointers for primitive geometry
        let vertexData = dynamicVB.contents().bindMemory(to: MJCMetalVertex.self, capacity: max_vertices)
        let indexData = dynamicIB.contents().bindMemory(to: UInt32.self, capacity: max_indices)

        var totalVertices = 0
        var totalIndices = 0

        // Get geom count
        let geomCount = Int(frame.geomCount())

        // Handle empty frame
        if geomCount == 0 {
            renderAxesGizmo(encoder: encoder, viewMatrix: viewMatrix, width: width, height: height)
            encoder.endEncoding()
            blitBrightnessSample(commandBuffer: commandBuffer, texture: drawable.texture,
                                 width: width, height: height, bufferIndex: currentBufferIndex)
            commandBuffer.present(drawable)
            commandBuffer.commit()
            return
        }

        // Get pointer to geoms array (using free function - member functions returning pointers not supported)
        guard let geomsPtr = MJFrameDataGetGeoms(frame) else {
            renderAxesGizmo(encoder: encoder, viewMatrix: viewMatrix, width: width, height: height)
            encoder.endEncoding()
            blitBrightnessSample(commandBuffer: commandBuffer, texture: drawable.texture,
                                 width: width, height: height, bufferIndex: currentBufferIndex)
            commandBuffer.present(drawable)
            commandBuffer.commit()
            return
        }

        // Build and set light buffer (once per frame, shared across all geoms)
        let emptyLight = MJCMetalLight(
            pos: .zero, _pad0: 0, dir: .zero, _pad1: 0,
            ambient: .zero, _pad2: 0, diffuse: .zero, _pad3: 0,
            specular: .zero, _pad4: 0, attenuation: .zero, cutoff: 0,
            exponent: 0, headlight: 0, directional: 0, _pad5: 0
        )
        var lightBuffer = MJCLightBuffer(
            lightCount: 0,
            _pad: (0, 0, 0),
            lights: (emptyLight, emptyLight, emptyLight, emptyLight,
                     emptyLight, emptyLight, emptyLight, emptyLight)
        )

        let lCount = Int(frame.lightCount())
        let lightsPtr = MJFrameDataGetLights(frame)
        if lCount > 0, let lightsPtr = lightsPtr {
            lightBuffer.lightCount = Int32(min(lCount, 8))
            for li in 0..<Int(lightBuffer.lightCount) {
                let src = lightsPtr[li]
                let light = MJCMetalLight(
                    pos: simd_float3(src.pos.0, src.pos.1, src.pos.2), _pad0: 0,
                    dir: simd_float3(src.dir.0, src.dir.1, src.dir.2), _pad1: 0,
                    ambient: simd_float3(src.ambient.0, src.ambient.1, src.ambient.2), _pad2: 0,
                    diffuse: simd_float3(src.diffuse.0, src.diffuse.1, src.diffuse.2), _pad3: 0,
                    specular: simd_float3(src.specular.0, src.specular.1, src.specular.2), _pad4: 0,
                    attenuation: simd_float3(src.attenuation.0, src.attenuation.1, src.attenuation.2),
                    cutoff: src.cutoff,
                    exponent: src.exponent,
                    headlight: src.headlight,
                    directional: src.directional,
                    _pad5: 0
                )
                switch li {
                case 0: lightBuffer.lights.0 = light
                case 1: lightBuffer.lights.1 = light
                case 2: lightBuffer.lights.2 = light
                case 3: lightBuffer.lights.3 = light
                case 4: lightBuffer.lights.4 = light
                case 5: lightBuffer.lights.5 = light
                case 6: lightBuffer.lights.6 = light
                case 7: lightBuffer.lights.7 = light
                default: break
                }
            }
        }
        encoder.setFragmentBytes(&lightBuffer, length: MemoryLayout<MJCLightBuffer>.stride, index: 2)

        // Render each geometry from ring buffer frame data
        var skippedGeoms = 0
        for i in 0..<geomCount {
            let geom = geomsPtr[i]
            let isMesh = geom.type == 7  // mjGEOM_MESH

            // Build model matrix from geom pose (shared by mesh and primitive paths)
            var modelMatrix = simd_float4x4(1.0)
            modelMatrix.columns.0 = simd_float4(geom.mat.0, geom.mat.3, geom.mat.6, 0)
            modelMatrix.columns.1 = simd_float4(geom.mat.1, geom.mat.4, geom.mat.7, 0)
            modelMatrix.columns.2 = simd_float4(geom.mat.2, geom.mat.5, geom.mat.8, 0)
            modelMatrix.columns.3 = simd_float4(geom.pos.0, geom.pos.1, geom.pos.2, 1)

            let emission = geom.emission
            let specular = geom.specular
            let shininess = min(max(geom.shininess, 1.0), 10.0)

            if isMesh {
                // --- Cached mesh path: bind pre-built buffers, no CPU vertex copying ---
                let dataid = Int(geom.dataid)
                let meshId = dataid >= 0 ? dataid / 2 : -1

                guard meshId >= 0, let cached = meshCache[meshId] else {
                    // Fallback: tiny placeholder box via dynamic buffer
                    if totalVertices + Self.max_vertices_per_geom > max_vertices ||
                       totalIndices + Self.max_indices_per_geom > max_indices {
                        skippedGeoms += 1; continue
                    }
                    var vertexCount = 0; var indexCount = 0
                    let color = simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3)
                    Self.generate_box(size: (0.02, 0.02, 0.02), color: color,
                                      vertices: vertexData.advanced(by: totalVertices),
                                      indices: indexData.advanced(by: totalIndices),
                                      vertexCount: &vertexCount, indexCount: &indexCount)
                    if vertexCount > 0 && indexCount > 0 {
                        var uniforms = MJCMetalUniforms(
                            modelMatrix: modelMatrix, viewMatrix: viewMatrix,
                            projectionMatrix: projMatrix, normal_matrix: Self.normal_matrix(from: modelMatrix),
                            lightPosition: simd_float3(0, 0, 10), _padding0: 0,
                            cameraPosition: eye, _padding1: 0,
                            color: simd_float4(1, 1, 1, 1),
                            emission: emission, specular: specular, shininess: shininess, checkerboardScale: 0)
                        encoder.setVertexBuffer(dynamicVB, offset: totalVertices * MemoryLayout<MJCMetalVertex>.stride, index: 0)
                        encoder.setVertexBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)
                        encoder.setFragmentBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)
                        encoder.drawIndexedPrimitives(type: .triangle, indexCount: indexCount, indexType: .uint32,
                                                      indexBuffer: dynamicIB,
                                                      indexBufferOffset: totalIndices * MemoryLayout<UInt32>.stride)
                        totalVertices += vertexCount; totalIndices += indexCount
                    }
                    continue
                }

                // Cached mesh: set geom color via uniforms (vertex color is white in cache)
                var uniforms = MJCMetalUniforms(
                    modelMatrix: modelMatrix, viewMatrix: viewMatrix,
                    projectionMatrix: projMatrix, normal_matrix: Self.normal_matrix(from: modelMatrix),
                    lightPosition: simd_float3(0, 0, 10), _padding0: 0,
                    cameraPosition: eye, _padding1: 0,
                    color: simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3),
                    emission: emission, specular: specular, shininess: shininess, checkerboardScale: 0)

                encoder.setVertexBuffer(cached.vertexBuffer, offset: 0, index: 0)
                encoder.setVertexBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)
                encoder.setFragmentBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)
                encoder.drawIndexedPrimitives(type: .triangle, indexCount: cached.indexCount, indexType: .uint32,
                                              indexBuffer: cached.indexBuffer, indexBufferOffset: 0)

            } else {
                // --- Dynamic primitive path: write to triple-buffered shared buffer ---
                if totalVertices + Self.max_vertices_per_geom > max_vertices ||
                   totalIndices + Self.max_indices_per_geom > max_indices {
                    skippedGeoms = geomCount - i
                    break
                }

                var vertexCount = 0
                var indexCount = 0
                Self.convert_geom_instance(
                    geom: geom,
                    vertices: vertexData.advanced(by: totalVertices),
                    indices: indexData.advanced(by: totalIndices),
                    vertexCount: &vertexCount,
                    indexCount: &indexCount
                )

                if vertexCount > 0 && indexCount > 0 {
                    // Primitives: color is baked into vertices, uniforms.color = white (passthrough)
                    let isPlane = geom.type == 0
                    var uniforms = MJCMetalUniforms(
                        modelMatrix: modelMatrix, viewMatrix: viewMatrix,
                        projectionMatrix: projMatrix, normal_matrix: Self.normal_matrix(from: modelMatrix),
                        lightPosition: simd_float3(0, 0, 10), _padding0: 0,
                        cameraPosition: eye, _padding1: 0,
                        color: simd_float4(1, 1, 1, 1),
                        emission: emission, specular: specular, shininess: shininess,
                        checkerboardScale: isPlane ? 1.0 : 0)

                    encoder.setVertexBuffer(dynamicVB, offset: totalVertices * MemoryLayout<MJCMetalVertex>.stride, index: 0)
                    encoder.setVertexBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)
                    encoder.setFragmentBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)
                    encoder.drawIndexedPrimitives(type: .triangle, indexCount: indexCount, indexType: .uint32,
                                                  indexBuffer: dynamicIB,
                                                  indexBufferOffset: totalIndices * MemoryLayout<UInt32>.stride)
                    totalVertices += vertexCount
                    totalIndices += indexCount
                }
            }
        }

        // Warn if geometries were skipped due to buffer overflow
        if skippedGeoms > 0 {
            let rendered = geomCount - skippedGeoms
            logger.warning("Skipped \(skippedGeoms)/\(geomCount) geometries due to buffer capacity (rendered \(rendered), vertices: \(totalVertices)/\(self.max_vertices), indices: \(totalIndices)/\(self.max_indices))")
        }

        renderAxesGizmo(encoder: encoder, viewMatrix: viewMatrix, width: width, height: height)

        encoder.endEncoding()

        blitBrightnessSample(commandBuffer: commandBuffer, texture: drawable.texture,
                             width: width, height: height, bufferIndex: currentBufferIndex)
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    // MARK: - Brightness Readback

    /// Blit a 4×4 pixel sample from the rendered texture for CPU brightness readback.
    /// Samples from the upper-center region where overlay text typically sits.
    private func blitBrightnessSample(commandBuffer: MTLCommandBuffer, texture: MTLTexture,
                                       width: Int, height: Int, bufferIndex: Int) {
        guard let blitEncoder = commandBuffer.makeBlitCommandEncoder() else { return }
        let sampleW = min(4, width)
        let sampleH = min(4, height)
        let sampleX = max(0, width / 2 - sampleW / 2)
        let sampleY = max(0, height / 4 - sampleH / 2)
        blitEncoder.copy(
            from: texture,
            sourceSlice: 0,
            sourceLevel: 0,
            sourceOrigin: MTLOrigin(x: sampleX, y: sampleY, z: 0),
            sourceSize: MTLSize(width: sampleW, height: sampleH, depth: 1),
            to: brightnessReadbackBuffers[bufferIndex],
            destinationOffset: 0,
            destinationBytesPerRow: sampleW * 4,
            destinationBytesPerImage: sampleW * sampleH * 4
        )
        blitEncoder.endEncoding()
        brightnessReadbackValid[bufferIndex] = true
    }

    // MARK: - Axes Gizmo

    /// Render a 3D XYZ orientation axes gizmo in the bottom-left corner.
    /// Rotates with the camera but stays fixed in the corner, never occluded by scene geometry.
    private func renderAxesGizmo(encoder: MTLRenderCommandEncoder, viewMatrix: simd_float4x4,
                                 width: Int, height: Int) {
        let gizmoSize = Int(Float(min(width, height)) * 0.15)
        let padding = 10

        // Set viewport to bottom-left corner
        encoder.setViewport(MTLViewport(
            originX: Double(padding),
            originY: Double(height - gizmoSize - padding),
            width: Double(gizmoSize),
            height: Double(gizmoSize),
            znear: 0, zfar: 1
        ))

        // Switch to gizmo depth state (always pass, no depth write)
        encoder.setDepthStencilState(gizmoDepthState)

        // Build rotation-only view matrix: keep camera rotation, replace translation
        // with fixed push-back so gizmo is always centered at a fixed distance
        var gizmoView = viewMatrix
        gizmoView.columns.3 = simd_float4(0, 0, -3, 1)

        // Orthographic projection (gizmo arrows extend ~0.9 units from origin)
        let orthoProj = Self.orthographic(left: -1.0, right: 1.0, bottom: -1.0, top: 1.0,
                                          near: 0.1, far: 10)

        // Simple directional light from camera direction for consistent gizmo shading
        let emptyLight = MJCMetalLight(
            pos: .zero, _pad0: 0, dir: .zero, _pad1: 0,
            ambient: .zero, _pad2: 0, diffuse: .zero, _pad3: 0,
            specular: .zero, _pad4: 0, attenuation: .zero, cutoff: 0,
            exponent: 0, headlight: 0, directional: 0, _pad5: 0
        )
        var gizmoLightBuffer = MJCLightBuffer(
            lightCount: 1,
            _pad: (0, 0, 0),
            lights: (
                MJCMetalLight(
                    pos: simd_float3(0, 0, 5), _pad0: 0,
                    dir: simd_float3(0, 0, -1), _pad1: 0,
                    ambient: simd_float3(0.4, 0.4, 0.4), _pad2: 0,
                    diffuse: simd_float3(0.8, 0.8, 0.8), _pad3: 0,
                    specular: simd_float3(0.3, 0.3, 0.3), _pad4: 0,
                    attenuation: simd_float3(1, 0, 0), cutoff: 180,
                    exponent: 0, headlight: 0, directional: 1, _pad5: 0
                ),
                emptyLight, emptyLight, emptyLight,
                emptyLight, emptyLight, emptyLight, emptyLight
            )
        )
        encoder.setFragmentBytes(&gizmoLightBuffer, length: MemoryLayout<MJCLightBuffer>.stride, index: 2)

        // Axis rotation matrices: rotate local-Z arrow geometry to point along target axis
        let axisRotations: [simd_float4x4] = [
            // X axis (red): rotate 90° around Y → local Z becomes +X
            simd_float4x4(columns: (
                simd_float4(0, 0, -1, 0),
                simd_float4(0, 1, 0, 0),
                simd_float4(1, 0, 0, 0),
                simd_float4(0, 0, 0, 1)
            )),
            // Y axis (green): rotate -90° around X → local Z becomes +Y
            simd_float4x4(columns: (
                simd_float4(1, 0, 0, 0),
                simd_float4(0, 0, -1, 0),
                simd_float4(0, 1, 0, 0),
                simd_float4(0, 0, 0, 1)
            )),
            // Z axis (blue): identity (arrow already along Z)
            simd_float4x4(1.0)
        ]

        // World-space axis directions for depth sorting
        let axisDirections: [simd_float3] = [
            simd_float3(1, 0, 0), simd_float3(0, 1, 0), simd_float3(0, 0, 1)
        ]

        // Camera -forward direction (row 2 of view matrix) for back-to-front sorting
        let viewRow2 = simd_float3(gizmoView.columns.0.z, gizmoView.columns.1.z, gizmoView.columns.2.z)

        // Sort axes back-to-front: most negative dot = furthest from camera = draw first
        var axisOrder = [0, 1, 2]
        axisOrder.sort { simd_dot(axisDirections[$0], viewRow2) < simd_dot(axisDirections[$1], viewRow2) }

        // Draw axes back-to-front using pre-built gizmo geometry
        encoder.setVertexBuffer(gizmoVertexBuffer, offset: 0, index: 0)
        for axisIdx in axisOrder {
            let modelMatrix = axisRotations[axisIdx]
            var uniforms = MJCMetalUniforms(
                modelMatrix: modelMatrix,
                viewMatrix: gizmoView,
                projectionMatrix: orthoProj,
                normal_matrix: Self.normal_matrix(from: modelMatrix),
                lightPosition: simd_float3(0, 0, 5), _padding0: 0,
                cameraPosition: simd_float3(0, 0, 3), _padding1: 0,
                color: simd_float4(1, 1, 1, 1),
                emission: 0.5, specular: 0.3, shininess: 5.0, checkerboardScale: 0
            )
            encoder.setVertexBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)
            encoder.setFragmentBytes(&uniforms, length: MemoryLayout<MJCMetalUniforms>.stride, index: 1)

            let range = gizmoIndexCounts[axisIdx]
            encoder.drawIndexedPrimitives(
                type: .triangle,
                indexCount: range.count,
                indexType: .uint32,
                indexBuffer: gizmoIndexBuffer,
                indexBufferOffset: range.offset * MemoryLayout<UInt32>.stride
            )
        }

        // Restore scene viewport and depth state
        encoder.setViewport(MTLViewport(
            originX: 0, originY: 0,
            width: Double(width), height: Double(height),
            znear: 0, zfar: 1
        ))
        encoder.setDepthStencilState(depth_state)
    }

    /// Build vertex and index buffers for the 3-axis gizmo (X=red, Y=green, Z=blue).
    /// Each axis is an arrow along +Z (shaft cylinder + cone arrowhead), to be rotated
    /// at render time via modelMatrix. All 3 axes share a single vertex/index buffer pair.
    private static func buildGizmoGeometry(device: MTLDevice) -> (vertexBuffer: MTLBuffer,
                                                                   indexBuffer: MTLBuffer,
                                                                   indexCounts: [(offset: Int, count: Int)])? {
        let segments = 8
        let shaftRadius: Float = 0.03
        let shaftLength: Float = 0.7
        let coneBaseRadius: Float = 0.08
        let coneHeight: Float = 0.2

        let axisColors: [simd_float4] = [
            simd_float4(0.9, 0.2, 0.2, 1),  // X - red
            simd_float4(0.2, 0.8, 0.2, 1),  // Y - green
            simd_float4(0.3, 0.4, 0.9, 1),  // Z - blue
        ]

        var allVertices: [MJCMetalVertex] = []
        var allIndices: [UInt32] = []
        var indexCounts: [(offset: Int, count: Int)] = []

        for axisIdx in 0..<3 {
            let color = axisColors[axisIdx]
            let baseVertex = UInt32(allVertices.count)
            let baseIndex = allIndices.count

            // --- Shaft: cylinder from z=0 to z=shaftLength ---
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = shaftRadius * cos(theta)
                let y = shaftRadius * sin(theta)
                let normal = simd_float3(cos(theta), sin(theta), 0)

                allVertices.append(MJCMetalVertex(position: simd_float3(x, y, 0),
                                                   normal: normal, texCoord: .zero, color: color))
                allVertices.append(MJCMetalVertex(position: simd_float3(x, y, shaftLength),
                                                   normal: normal, texCoord: .zero, color: color))
            }

            for s in 0..<segments {
                let c = baseVertex + UInt32(s * 2)
                allIndices.append(contentsOf: [c, c + 2, c + 1, c + 1, c + 2, c + 3])
            }

            // --- Arrowhead: cone from z=shaftLength to z=shaftLength+coneHeight ---
            let coneApexIdx = UInt32(allVertices.count)
            allVertices.append(MJCMetalVertex(position: simd_float3(0, 0, shaftLength + coneHeight),
                                               normal: simd_float3(0, 0, 1), texCoord: .zero, color: color))

            let coneRingBase = UInt32(allVertices.count)
            let slopeAngle = atan2(coneBaseRadius, coneHeight)
            let nz = sin(slopeAngle)
            let nr = cos(slopeAngle)

            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = coneBaseRadius * cos(theta)
                let y = coneBaseRadius * sin(theta)
                let normal = simd_float3(nr * cos(theta), nr * sin(theta), nz)
                allVertices.append(MJCMetalVertex(position: simd_float3(x, y, shaftLength),
                                                   normal: normal, texCoord: .zero, color: color))
            }

            for s in 0..<segments {
                allIndices.append(contentsOf: [coneApexIdx, coneRingBase + UInt32(s),
                                               coneRingBase + UInt32(s + 1)])
            }

            // Cone base disk
            let coneCenterIdx = UInt32(allVertices.count)
            allVertices.append(MJCMetalVertex(position: simd_float3(0, 0, shaftLength),
                                               normal: simd_float3(0, 0, -1), texCoord: .zero, color: color))
            let coneDiskRing = UInt32(allVertices.count)
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = coneBaseRadius * cos(theta)
                let y = coneBaseRadius * sin(theta)
                allVertices.append(MJCMetalVertex(position: simd_float3(x, y, shaftLength),
                                                   normal: simd_float3(0, 0, -1), texCoord: .zero, color: color))
            }

            for s in 0..<segments {
                allIndices.append(contentsOf: [coneCenterIdx, coneDiskRing + UInt32(s + 1),
                                               coneDiskRing + UInt32(s)])
            }

            indexCounts.append((offset: baseIndex, count: allIndices.count - baseIndex))
        }

        guard let vBuf = device.makeBuffer(bytes: allVertices,
                                            length: allVertices.count * MemoryLayout<MJCMetalVertex>.stride,
                                            options: .storageModeShared),
              let iBuf = device.makeBuffer(bytes: allIndices,
                                            length: allIndices.count * MemoryLayout<UInt32>.stride,
                                            options: .storageModeShared) else {
            return nil
        }

        return (vBuf, iBuf, indexCounts)
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

    private static func orthographic(left: Float, right: Float, bottom: Float, top: Float,
                                     near: Float, far: Float) -> simd_float4x4 {
        let sx = 2.0 / (right - left)
        let sy = 2.0 / (top - bottom)
        let sz = 1.0 / (near - far)
        let tx = -(right + left) / (right - left)
        let ty = -(top + bottom) / (top - bottom)
        let tz = near / (near - far)

        return simd_float4x4(columns: (
            simd_float4(sx, 0, 0, 0),
            simd_float4(0, sy, 0, 0),
            simd_float4(0, 0, sz, 0),
            simd_float4(tx, ty, tz, 1)
        ))
    }

    private static func look_at(eye: simd_float3, center: simd_float3, up: simd_float3) -> simd_float4x4 {
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
    private static func normal_matrix(from modelMatrix: simd_float4x4) -> simd_float4x4 {
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
    private static func convert_geom_instance(geom: MJGeomInstance,
                                            vertices: UnsafeMutablePointer<MJCMetalVertex>,
                                            indices: UnsafeMutablePointer<UInt32>,
                                            vertexCount: inout Int,
                                            indexCount: inout Int) {
        let color = simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3)
        let size = (geom.size.0, geom.size.1, geom.size.2)

        switch geom.type {
        case 0: // mjGEOM_PLANE
            generate_plane(size: size, color: color, vertices: vertices, indices: indices,
                         vertexCount: &vertexCount, indexCount: &indexCount)

        case 2: // mjGEOM_SPHERE
            generate_sphere(radius: geom.size.0, color: color, vertices: vertices, indices: indices,
                          vertexCount: &vertexCount, indexCount: &indexCount)

        case 3: // mjGEOM_CAPSULE
            generate_capsule(radius: geom.size.0, halfLength: geom.size.2, color: color,
                           vertices: vertices, indices: indices,
                           vertexCount: &vertexCount, indexCount: &indexCount)

        case 5: // mjGEOM_CYLINDER
            generate_cylinder(radius: geom.size.0, halfLength: geom.size.2, color: color,
                            vertices: vertices, indices: indices,
                            vertexCount: &vertexCount, indexCount: &indexCount)

        case 6: // mjGEOM_BOX
            generate_box(size: size, color: color, vertices: vertices, indices: indices,
                       vertexCount: &vertexCount, indexCount: &indexCount)

        default:
            // Placeholder cube for unsupported types
            generate_box(size: (0.02, 0.02, 0.02), color: color, vertices: vertices, indices: indices,
                       vertexCount: &vertexCount, indexCount: &indexCount)
        }
    }

    // MARK: - Geometry Generators

    /// Generate a single-quad ground plane. Checkerboard pattern is computed procedurally
    /// in the fragment shader using worldPosition, so pattern density stays constant
    /// regardless of zoom level (like MuJoCo's OpenGL renderer).
    private static func generate_plane(size: (Float, Float, Float), color: simd_float4,
                                      vertices: UnsafeMutablePointer<MJCMetalVertex>,
                                      indices: UnsafeMutablePointer<UInt32>,
                                      vertexCount: inout Int, indexCount: inout Int) {
        let sx = size.0 > 0 ? size.0 : Float(500.0)
        let sy = size.1 > 0 ? size.1 : Float(500.0)
        let normal = simd_float3(0, 0, 1)

        vertices[0] = MJCMetalVertex(position: simd_float3(-sx, -sy, 0), normal: normal, texCoord: .zero, color: color)
        vertices[1] = MJCMetalVertex(position: simd_float3( sx, -sy, 0), normal: normal, texCoord: .zero, color: color)
        vertices[2] = MJCMetalVertex(position: simd_float3( sx,  sy, 0), normal: normal, texCoord: .zero, color: color)
        vertices[3] = MJCMetalVertex(position: simd_float3(-sx,  sy, 0), normal: normal, texCoord: .zero, color: color)

        indices[0] = 0; indices[1] = 1; indices[2] = 2
        indices[3] = 0; indices[4] = 2; indices[5] = 3

        vertexCount = 4
        indexCount = 6
    }

    /// Generate a UV sphere with the given radius and color.
    /// Tessellation: 16 segments × 12 rings provides smooth silhouettes while keeping
    /// vertex count reasonable for real-time rendering of typical MuJoCo scenes.
    private static func generate_sphere(radius: Float, color: simd_float4,
                                       vertices: UnsafeMutablePointer<MJCMetalVertex>,
                                       indices: UnsafeMutablePointer<UInt32>,
                                       vertexCount: inout Int, indexCount: inout Int) {
        let segments = 16  // Horizontal divisions (longitude)
        let rings = 12     // Vertical divisions (latitude)
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

                vertices[vCount] = MJCMetalVertex(position: pos, normal: normal, texCoord: texCoord, color: color)
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

    private static func generate_box(size: (Float, Float, Float), color: simd_float4,
                                    vertices: UnsafeMutablePointer<MJCMetalVertex>,
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
                vertices[vCount] = MJCMetalVertex(
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

    /// Generate a cylinder with the given radius, half-length, and color.
    /// Tessellation: 16 segments around the circumference for smooth appearance.
    private static func generate_cylinder(radius: Float, halfLength: Float, color: simd_float4,
                                         vertices: UnsafeMutablePointer<MJCMetalVertex>,
                                         indices: UnsafeMutablePointer<UInt32>,
                                         vertexCount: inout Int, indexCount: inout Int) {
        let segments = 16  // Circumference divisions
        var vCount = 0
        var iCount = 0

        for s in 0...segments {
            let theta = 2.0 * Float.pi * Float(s) / Float(segments)
            let x = radius * cos(theta)
            let y = radius * sin(theta)
            let normal = simd_float3(cos(theta), sin(theta), 0)

            // Bottom vertex
            vertices[vCount] = MJCMetalVertex(
                position: simd_float3(x, y, -halfLength),
                normal: normal,
                texCoord: simd_float2(Float(s) / Float(segments), 0),
                color: color
            )
            vCount += 1

            // Top vertex
            vertices[vCount] = MJCMetalVertex(
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

    /// Generate a capsule (cylinder with hemisphere caps) with the given radius, half-length, and color.
    /// Tessellation: 16 segments × 8 rings per hemisphere for smooth caps.
    private static func generate_capsule(radius: Float, halfLength: Float, color: simd_float4,
                                        vertices: UnsafeMutablePointer<MJCMetalVertex>,
                                        indices: UnsafeMutablePointer<UInt32>,
                                        vertexCount: inout Int, indexCount: inout Int) {
        // Start with cylinder
        generate_cylinder(radius: radius, halfLength: halfLength, color: color,
                        vertices: vertices, indices: indices,
                        vertexCount: &vertexCount, indexCount: &indexCount)

        // Add hemisphere caps
        let segments = 16  // Circumference divisions (matches cylinder)
        let rings = 8      // Latitude divisions per hemisphere

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

                vertices[vertexCount] = MJCMetalVertex(
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

                vertices[vertexCount] = MJCMetalVertex(
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

public enum MJCRenderError: Error {
    case command_queueCreationFailed
    case shaderCompilationFailed
    case shaderFunctionNotFound
    case pipelineCreationFailed
    case depth_stateCreationFailed
    case bufferAllocationFailed
}
