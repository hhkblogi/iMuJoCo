// mjc_offscreen_render.swift
// Simplified Metal renderer for offscreen video capture.
// Renders MuJoCo scenes to a buffer-backed MTLTexture for zero-copy CPU readback.
// On Apple Silicon UMA, the GPU writes directly into linear MTLBuffer memory that
// CoreGraphics reads without any copy (analogous to NvBufSurface on NVIDIA Jetson).
// Designed to run on a dedicated capture thread, separate from the display renderer.

import Metal
import simd
import os.log
import MJCPhysicsRuntime

private let logger = Logger(subsystem: "com.mujoco.video", category: "OffscreenRender")

// MARK: - Vertex Type (must match shader VertexIn layout, 64 bytes)

struct MJCOffscreenVertex {
    var position: simd_float3
    var normal: simd_float3
    var texCoord: simd_float2
    var _padding: simd_float2
    var color: simd_float4

    init(position: simd_float3, normal: simd_float3,
         texCoord: simd_float2 = .zero, color: simd_float4 = simd_float4(1, 1, 1, 1)) {
        self.position = position
        self.normal = normal
        self.texCoord = texCoord
        self._padding = .zero
        self.color = color
    }
}

// MARK: - Shader Uniform Types (must match MJCMetalRender's layout)

struct MJCOffscreenFrameUniforms {
    var viewMatrix: simd_float4x4
    var projectionMatrix: simd_float4x4
    var cameraPosition: simd_float3
    var _padding: Float
}

struct MJCOffscreenInstanceData {
    var modelMatrix: simd_float4x4
    var normalMatrix: simd_float4x4
    var color: simd_float4
    var emission: Float
    var specular: Float
    var shininess: Float
    var checkerboardScale: Float
}

struct MJCOffscreenLight {
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

struct MJCOffscreenLightBuffer {
    var lightCount: Int32
    var _pad: (Int32, Int32, Int32)
    var lights: (MJCOffscreenLight, MJCOffscreenLight, MJCOffscreenLight, MJCOffscreenLight,
                 MJCOffscreenLight, MJCOffscreenLight, MJCOffscreenLight, MJCOffscreenLight)
}

// MARK: - Offscreen Render

/// Metal-based offscreen renderer for MuJoCo scenes.
///
/// Creates its own command queue, pipeline state, and mesh/texture caches so it can
/// run independently on a capture thread without interfering with the display renderer.
/// Renders to `.rgba8Unorm` texture with `.storageModeShared` for zero-copy CPU readback.
public final class MJCOffscreenRender {
    private let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private let pipelineState: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState

    public private(set) var width: Int
    public private(set) var height: Int

    // Buffer-backed color texture: Metal renders directly into the MTLBuffer's
    // linear memory, eliminating the getBytes() un-tiling copy.  On Apple Silicon
    // UMA, the GPU and CPU share the same physical pages — truly zero-copy from
    // render output to JPEG encoder input (analogous to NvBufSurface on Jetson).
    private var colorBuffer: MTLBuffer!
    private var colorTexture: MTLTexture!
    private var depthTexture: MTLTexture!

    // Mesh cache
    private struct CachedMesh {
        let vertexBuffer: MTLBuffer
        let indexBuffer: MTLBuffer
        let vertexCount: Int
        let indexCount: Int
    }
    private var meshCache: [Int: CachedMesh] = [:]
    private var meshCacheBuilt = false
    private var meshCacheSourcePtr: UnsafeRawPointer?

    // Texture cache
    private var textureCache: [Int: MTLTexture] = [:]
    private var matTexMap: [Int32: Int] = [:]
    private var placeholderTexture: MTLTexture!
    private var samplerState: MTLSamplerState!
    private var textureCacheBuilt = false
    private var textureCacheSourcePtr: UnsafeRawPointer?

    // Dynamic buffers (single-buffered — synchronous rendering at low fps)
    private var dynamicVertexBuffer: MTLBuffer!
    private var dynamicIndexBuffer: MTLBuffer!
    private var instanceBuffer: MTLBuffer!

    private let maxVertices = 32 * 1024
    private let maxIndices = 192 * 1024
    private let maxInstances = 4096
    private static let maxVerticesPerGeom = 1000
    private static let maxIndicesPerGeom = 6000

    // Instancing tracking
    private struct DrawGroupKey: Hashable {
        let meshId: Int
        let texId: Int
    }
    private var perGroupCount: [DrawGroupKey: Int] = [:]
    private var perGroupWriteIdx: [DrawGroupKey: Int] = [:]
    private var activeGroups: [DrawGroupKey] = []
    private var primDrawList: [(instanceIndex: Int, vertexOffset: Int, indexOffset: Int, indexCount: Int)] = []
    private var cachedMeshIds: [Int] = []
    private var meshIdValid: [Bool] = []

    // MARK: - Initialization

    public init(device: MTLDevice, width: Int, height: Int) throws {
        self.device = device
        self.width = width
        self.height = height

        guard let queue = device.makeCommandQueue() else {
            throw MJCOffscreenRenderError.initFailed("Command queue creation failed")
        }
        self.commandQueue = queue

        // Load shaders from main bundle (same metallib as display renderer)
        guard let library = device.makeDefaultLibrary() else {
            throw MJCOffscreenRenderError.initFailed("Metal library not found")
        }
        guard let vertexFunc = library.makeFunction(name: "vertexMain"),
              let fragmentFunc = library.makeFunction(name: "fragmentMain") else {
            throw MJCOffscreenRenderError.initFailed("Shader functions not found")
        }

        // Vertex descriptor (identical to display renderer)
        let vertexDescriptor = MTLVertexDescriptor()
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        vertexDescriptor.attributes[1].format = .float3
        vertexDescriptor.attributes[1].offset = 16
        vertexDescriptor.attributes[1].bufferIndex = 0
        vertexDescriptor.attributes[2].format = .float2
        vertexDescriptor.attributes[2].offset = 32
        vertexDescriptor.attributes[2].bufferIndex = 0
        vertexDescriptor.attributes[3].format = .float4
        vertexDescriptor.attributes[3].offset = 48
        vertexDescriptor.attributes[3].bufferIndex = 0
        vertexDescriptor.layouts[0].stride = MemoryLayout<MJCOffscreenVertex>.stride
        vertexDescriptor.layouts[0].stepFunction = .perVertex

        // Pipeline — use .rgba8Unorm for direct RGBA output (no B↔R swizzle needed)
        let pipelineDesc = MTLRenderPipelineDescriptor()
        pipelineDesc.vertexFunction = vertexFunc
        pipelineDesc.fragmentFunction = fragmentFunc
        pipelineDesc.vertexDescriptor = vertexDescriptor
        pipelineDesc.colorAttachments[0].pixelFormat = .rgba8Unorm
        pipelineDesc.depthAttachmentPixelFormat = .depth32Float
        pipelineDesc.colorAttachments[0].isBlendingEnabled = true
        pipelineDesc.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        pipelineDesc.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        pipelineDesc.colorAttachments[0].sourceAlphaBlendFactor = .one
        pipelineDesc.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha

        self.pipelineState = try device.makeRenderPipelineState(descriptor: pipelineDesc)

        // Depth state
        let depthDesc = MTLDepthStencilDescriptor()
        depthDesc.depthCompareFunction = .less
        depthDesc.isDepthWriteEnabled = true
        guard let ds = device.makeDepthStencilState(descriptor: depthDesc) else {
            throw MJCOffscreenRenderError.initFailed("Depth state creation failed")
        }
        self.depthState = ds

        // Placeholder texture (1x1 white RGBA)
        let placeholderDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rgba8Unorm, width: 1, height: 1, mipmapped: false)
        placeholderDesc.usage = .shaderRead
        guard let placeholder = device.makeTexture(descriptor: placeholderDesc) else {
            throw MJCOffscreenRenderError.initFailed("Placeholder texture failed")
        }
        let white: [UInt8] = [255, 255, 255, 255]
        placeholder.replace(region: MTLRegionMake2D(0, 0, 1, 1),
                            mipmapLevel: 0, withBytes: white, bytesPerRow: 4)
        self.placeholderTexture = placeholder

        // Sampler
        let samplerDesc = MTLSamplerDescriptor()
        samplerDesc.minFilter = .nearest
        samplerDesc.magFilter = .nearest
        samplerDesc.sAddressMode = .clampToEdge
        samplerDesc.tAddressMode = .clampToEdge
        guard let sampler = device.makeSamplerState(descriptor: samplerDesc) else {
            throw MJCOffscreenRenderError.initFailed("Sampler creation failed")
        }
        self.samplerState = sampler

        // Dynamic buffers
        guard let vBuf = device.makeBuffer(length: maxVertices * MemoryLayout<MJCOffscreenVertex>.stride,
                                            options: .storageModeShared),
              let iBuf = device.makeBuffer(length: maxIndices * MemoryLayout<UInt32>.stride,
                                            options: .storageModeShared),
              let instBuf = device.makeBuffer(length: maxInstances * MemoryLayout<MJCOffscreenInstanceData>.stride,
                                               options: .storageModeShared) else {
            throw MJCOffscreenRenderError.initFailed("Buffer allocation failed")
        }
        self.dynamicVertexBuffer = vBuf
        self.dynamicIndexBuffer = iBuf
        self.instanceBuffer = instBuf

        // Create offscreen textures
        try createTextures()
    }

    // MARK: - Texture Management

    private func createTextures() throws {
        let bytesPerRow = width * 4
        let bufferSize = bytesPerRow * height

        // Allocate a shared MTLBuffer for the color attachment.
        // Metal renders directly into this linear buffer — no un-tiling copy needed.
        guard let buffer = device.makeBuffer(length: bufferSize, options: .storageModeShared) else {
            throw MJCOffscreenRenderError.initFailed("Color buffer creation failed")
        }
        self.colorBuffer = buffer

        // Create a 2D texture view over the buffer's linear memory.
        let colorDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rgba8Unorm, width: width, height: height, mipmapped: false)
        colorDesc.usage = [.renderTarget, .shaderRead]
        colorDesc.storageMode = .shared
        guard let color = buffer.makeTexture(
            descriptor: colorDesc, offset: 0, bytesPerRow: bytesPerRow) else {
            throw MJCOffscreenRenderError.initFailed("Color texture creation failed")
        }
        self.colorTexture = color

        let depthDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .depth32Float, width: width, height: height, mipmapped: false)
        depthDesc.usage = .renderTarget
        depthDesc.storageMode = .private
        guard let depth = device.makeTexture(descriptor: depthDesc) else {
            throw MJCOffscreenRenderError.initFailed("Depth texture creation failed")
        }
        self.depthTexture = depth
    }

    /// Update the render resolution.
    public func resize(newWidth: Int, newHeight: Int) throws {
        guard newWidth != width || newHeight != height else { return }
        self.width = newWidth
        self.height = newHeight
        try createTextures()
    }

    // MARK: - Mesh Cache

    private func invalidateMeshCache() {
        meshCache.removeAll()
        meshCacheBuilt = false
        meshCacheSourcePtr = nil
        cachedMeshIds.removeAll()
        meshIdValid.removeAll()
    }

    private func buildMeshCache(meshData: MJMeshData) {
        guard !meshCacheBuilt else { return }
        guard let meshInfoPtr = MJMeshDataGetMeshes(meshData),
              let meshVertPtr = MJMeshDataGetVertices(meshData),
              let meshFacePtr = MJMeshDataGetFaces(meshData) else { return }
        meshCacheBuilt = true

        let meshCount = Int(meshData.meshCount())
        for meshId in 0..<meshCount {
            let info = meshInfoPtr[meshId]
            let nv = Int(info.vertexCount)
            let nf = Int(info.faceCount)
            guard nv > 0, nf > 0 else { continue }

            let vertexStride = MemoryLayout<MJCOffscreenVertex>.stride
            guard let vBuf = device.makeBuffer(length: nv * vertexStride, options: .storageModeShared) else { continue }
            let vPtr = vBuf.contents().bindMemory(to: MJCOffscreenVertex.self, capacity: nv)
            let vertBase = Int(info.vertexOffset) * Int(MJ_VERTEX_FLOATS)
            for v in 0..<nv {
                let srcIdx = vertBase + v * Int(MJ_VERTEX_FLOATS)
                let pos = simd_float3(meshVertPtr[srcIdx], meshVertPtr[srcIdx + 1], meshVertPtr[srcIdx + 2])
                let normal = simd_float3(meshVertPtr[srcIdx + 3], meshVertPtr[srcIdx + 4], meshVertPtr[srcIdx + 5])
                let uv = simd_float2(meshVertPtr[srcIdx + 6], meshVertPtr[srcIdx + 7])
                vPtr[v] = MJCOffscreenVertex(position: pos, normal: normal, texCoord: uv)
            }

            let indexCount = nf * 3
            guard let iBuf = device.makeBuffer(length: indexCount * MemoryLayout<UInt32>.stride,
                                                options: .storageModeShared) else { continue }
            let iPtr = iBuf.contents().bindMemory(to: UInt32.self, capacity: indexCount)
            let faceBase = Int(info.faceOffset) * 3
            for f in 0..<indexCount {
                iPtr[f] = UInt32(meshFacePtr[faceBase + f])
            }

            meshCache[meshId] = CachedMesh(vertexBuffer: vBuf, indexBuffer: iBuf,
                                             vertexCount: nv, indexCount: indexCount)
        }

        let maxId = meshCache.keys.max() ?? -1
        cachedMeshIds = meshCache.keys.sorted()
        meshIdValid = [Bool](repeating: false, count: maxId + 1)
        for mid in cachedMeshIds { meshIdValid[mid] = true }
        activeGroups.reserveCapacity(maxId + 1)
        primDrawList.reserveCapacity(256)
    }

    // MARK: - Texture Cache

    private func invalidateTextureCache() {
        textureCache.removeAll()
        matTexMap.removeAll()
        textureCacheBuilt = false
        textureCacheSourcePtr = nil
    }

    private func buildTextureCache(textureData: MJTextureData) {
        guard !textureCacheBuilt else { return }
        guard let texInfoPtr = MJTextureDataGetTextures(textureData),
              let pixelPtr = MJTextureDataGetPixels(textureData) else { return }

        let texCount = Int(textureData.textureCount())
        for i in 0..<texCount {
            let info = texInfoPtr[i]
            let w = Int(info.width), h = Int(info.height)
            guard w > 0, h > 0 else { continue }

            let desc = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: .rgba8Unorm, width: w, height: h, mipmapped: false)
            desc.usage = .shaderRead
            guard let tex = device.makeTexture(descriptor: desc) else { continue }
            let srcPtr = pixelPtr.advanced(by: Int(info.dataOffset))
            tex.replace(region: MTLRegionMake2D(0, 0, w, h),
                        mipmapLevel: 0, withBytes: srcPtr, bytesPerRow: w * 4)
            textureCache[i] = tex
        }

        if let matTexIdPtr = MJTextureDataGetMatTexId(textureData) {
            let matCount = Int(textureData.materialCount())
            for m in 0..<matCount {
                let texId = matTexIdPtr[m]
                if texId >= 0 { matTexMap[Int32(m)] = Int(texId) }
            }
        }
        textureCacheBuilt = true
    }

    // MARK: - Render

    /// Render the scene and return raw RGBA pixel data.
    /// This is synchronous — it blocks until the GPU finishes.
    ///
    /// - Returns: RGBA pixel data (width * height * 4 bytes), or nil on failure.
    public func render(frame: MJFrameData,
                       meshData: MJMeshData? = nil,
                       textureData: MJTextureData? = nil) -> Data? {
        // Update mesh cache
        if let md = meshData {
            let newPtr = UnsafeRawPointer(Unmanaged.passUnretained(md).toOpaque())
            if !meshCacheBuilt || meshCacheSourcePtr != newPtr {
                invalidateMeshCache()
                buildMeshCache(meshData: md)
                meshCacheSourcePtr = newPtr
            }
        } else if meshCacheBuilt {
            invalidateMeshCache()
        }

        // Update texture cache
        if let td = textureData {
            let newPtr = UnsafeRawPointer(Unmanaged.passUnretained(td).toOpaque())
            if !textureCacheBuilt || textureCacheSourcePtr != newPtr {
                invalidateTextureCache()
                buildTextureCache(textureData: td)
                textureCacheSourcePtr = newPtr
            }
        } else if textureCacheBuilt {
            invalidateTextureCache()
        }

        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return nil }

        // Render pass descriptor
        let passDesc = MTLRenderPassDescriptor()
        passDesc.colorAttachments[0].texture = colorTexture
        passDesc.colorAttachments[0].loadAction = .clear
        passDesc.colorAttachments[0].storeAction = .store
        passDesc.colorAttachments[0].clearColor = MTLClearColor(red: 0.15, green: 0.18, blue: 0.22, alpha: 1.0)
        passDesc.depthAttachment.texture = depthTexture
        passDesc.depthAttachment.loadAction = .clear
        passDesc.depthAttachment.storeAction = .dontCare
        passDesc.depthAttachment.clearDepth = 1.0

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: passDesc) else {
            commandBuffer.commit()
            return nil
        }

        encoder.setRenderPipelineState(pipelineState)
        encoder.setDepthStencilState(depthState)
        encoder.setCullMode(.none)

        let geomCount = Int(frame.geomCount())
        if geomCount == 0 {
            encoder.endEncoding()
            commandBuffer.commit()
            commandBuffer.waitUntilCompleted()
            return readbackPixels()
        }

        guard let geomsPtr = MJFrameDataGetGeoms(frame) else {
            encoder.endEncoding()
            commandBuffer.commit()
            commandBuffer.waitUntilCompleted()
            return readbackPixels()
        }

        // Camera setup
        let aspect = Float(width) / Float(height)
        let fovy = Float(45.0 * .pi / 180.0)
        let azRad = frame.cameraAzimuth() * .pi / 180.0
        let elRad = frame.cameraElevation() * .pi / 180.0
        let dist = frame.cameraDistance()
        let lookatX = frame.cameraLookatX()
        let lookatY = frame.cameraLookatY()
        let lookatZ = frame.cameraLookatZ()

        let eye = simd_float3(
            lookatX + dist * sin(azRad) * cos(elRad),
            lookatY + dist * cos(azRad) * cos(elRad),
            lookatZ + dist * sin(elRad))
        let center = simd_float3(lookatX, lookatY, lookatZ)
        let viewMatrix = Self.lookAt(eye: eye, center: center, up: simd_float3(0, 0, 1))
        let projMatrix = Self.perspective(fovy: fovy, aspect: aspect, near: 0.01, far: 500.0)

        var frameUniforms = MJCOffscreenFrameUniforms(
            viewMatrix: viewMatrix, projectionMatrix: projMatrix,
            cameraPosition: eye, _padding: 0)
        encoder.setVertexBytes(&frameUniforms, length: MemoryLayout<MJCOffscreenFrameUniforms>.stride, index: 1)
        encoder.setFragmentBytes(&frameUniforms, length: MemoryLayout<MJCOffscreenFrameUniforms>.stride, index: 1)

        // Lights
        let emptyLight = MJCOffscreenLight(
            pos: .zero, _pad0: 0, dir: .zero, _pad1: 0,
            ambient: .zero, _pad2: 0, diffuse: .zero, _pad3: 0,
            specular: .zero, _pad4: 0, attenuation: .zero, cutoff: 0,
            exponent: 0, headlight: 0, directional: 0, _pad5: 0)
        var lightBuffer = MJCOffscreenLightBuffer(
            lightCount: 0, _pad: (0, 0, 0),
            lights: (emptyLight, emptyLight, emptyLight, emptyLight,
                     emptyLight, emptyLight, emptyLight, emptyLight))

        let lCount = Int(frame.lightCount())
        let lightsPtr = MJFrameDataGetLights(frame)
        if lCount > 0, let lightsPtr = lightsPtr {
            lightBuffer.lightCount = Int32(min(lCount, 8))
            for li in 0..<Int(lightBuffer.lightCount) {
                let src = lightsPtr[li]
                let light = MJCOffscreenLight(
                    pos: simd_float3(src.pos.0, src.pos.1, src.pos.2), _pad0: 0,
                    dir: simd_float3(src.dir.0, src.dir.1, src.dir.2), _pad1: 0,
                    ambient: simd_float3(src.ambient.0, src.ambient.1, src.ambient.2), _pad2: 0,
                    diffuse: simd_float3(src.diffuse.0, src.diffuse.1, src.diffuse.2), _pad3: 0,
                    specular: simd_float3(src.specular.0, src.specular.1, src.specular.2), _pad4: 0,
                    attenuation: simd_float3(src.attenuation.0, src.attenuation.1, src.attenuation.2),
                    cutoff: src.cutoff, exponent: src.exponent,
                    headlight: src.headlight, directional: src.directional, _pad5: 0)
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
        encoder.setFragmentBytes(&lightBuffer, length: MemoryLayout<MJCOffscreenLightBuffer>.stride, index: 2)

        // Prepare buffers
        let vertexData = dynamicVertexBuffer.contents().bindMemory(to: MJCOffscreenVertex.self, capacity: maxVertices)
        let indexData = dynamicIndexBuffer.contents().bindMemory(to: UInt32.self, capacity: maxIndices)
        let instanceData = instanceBuffer.contents().bindMemory(to: MJCOffscreenInstanceData.self, capacity: maxInstances)

        var totalVertices = 0
        var totalIndices = 0
        let meshIdTableSize = meshIdValid.count

        // Counting pass
        perGroupCount.removeAll(keepingCapacity: true)
        perGroupWriteIdx.removeAll(keepingCapacity: true)
        activeGroups.removeAll(keepingCapacity: true)
        primDrawList.removeAll(keepingCapacity: true)

        var totalMeshInstances = 0

        for i in 0..<geomCount {
            let geom = geomsPtr[i]
            if geom.type == 7 {
                let dataid = Int(geom.dataid)
                let meshId = dataid >= 0 ? dataid / 2 : -1
                if meshId >= 0 && meshId < meshIdTableSize && meshIdValid[meshId] {
                    let texId = matTexMap[geom.matid] ?? -1
                    let key = DrawGroupKey(meshId: meshId, texId: texId)
                    if perGroupCount[key] == nil {
                        activeGroups.append(key)
                        perGroupCount[key] = 0
                    }
                    perGroupCount[key]! += 1
                    totalMeshInstances += 1
                }
            }
        }

        var offset = 0
        for key in activeGroups {
            perGroupWriteIdx[key] = offset
            offset += perGroupCount[key]!
        }
        let primBaseInstance = totalMeshInstances

        // Write pass
        var primInstanceIdx = primBaseInstance

        for i in 0..<geomCount {
            let geom = geomsPtr[i]
            var modelMatrix = simd_float4x4(1.0)
            modelMatrix.columns.0 = simd_float4(geom.mat.0, geom.mat.3, geom.mat.6, 0)
            modelMatrix.columns.1 = simd_float4(geom.mat.1, geom.mat.4, geom.mat.7, 0)
            modelMatrix.columns.2 = simd_float4(geom.mat.2, geom.mat.5, geom.mat.8, 0)
            modelMatrix.columns.3 = simd_float4(geom.pos.0, geom.pos.1, geom.pos.2, 1)

            if geom.type == 7 {
                let dataid = Int(geom.dataid)
                let meshId = dataid >= 0 ? dataid / 2 : -1
                if meshId >= 0 && meshId < meshIdTableSize && meshIdValid[meshId] {
                    let texId = matTexMap[geom.matid] ?? -1
                    let key = DrawGroupKey(meshId: meshId, texId: texId)
                    let idx = perGroupWriteIdx[key]!
                    guard idx < maxInstances else { continue }
                    instanceData[idx] = MJCOffscreenInstanceData(
                        modelMatrix: modelMatrix,
                        normalMatrix: Self.normalMatrix(from: modelMatrix),
                        color: simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3),
                        emission: geom.emission, specular: geom.specular,
                        shininess: min(max(geom.shininess, 1.0), 10.0), checkerboardScale: 0)
                    perGroupWriteIdx[key] = idx + 1
                } else {
                    guard primInstanceIdx < maxInstances else { continue }
                    guard totalVertices + Self.maxVerticesPerGeom <= maxVertices,
                          totalIndices + Self.maxIndicesPerGeom <= maxIndices else { continue }
                    var vc = 0; var ic = 0
                    let color = simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3)
                    Self.generateBox(size: (0.02, 0.02, 0.02), color: color,
                                     vertices: vertexData.advanced(by: totalVertices),
                                     indices: indexData.advanced(by: totalIndices),
                                     vertexCount: &vc, indexCount: &ic)
                    guard vc > 0, ic > 0 else { continue }
                    instanceData[primInstanceIdx] = MJCOffscreenInstanceData(
                        modelMatrix: modelMatrix, normalMatrix: Self.normalMatrix(from: modelMatrix),
                        color: simd_float4(1, 1, 1, 1), emission: geom.emission,
                        specular: geom.specular, shininess: min(max(geom.shininess, 1.0), 10.0),
                        checkerboardScale: 0)
                    primDrawList.append((instanceIndex: primInstanceIdx, vertexOffset: totalVertices,
                                         indexOffset: totalIndices, indexCount: ic))
                    primInstanceIdx += 1; totalVertices += vc; totalIndices += ic
                }
            } else {
                guard primInstanceIdx < maxInstances else { continue }
                guard totalVertices + Self.maxVerticesPerGeom <= maxVertices,
                      totalIndices + Self.maxIndicesPerGeom <= maxIndices else { continue }
                var vc = 0; var ic = 0
                Self.convertGeom(geom: geom,
                                 vertices: vertexData.advanced(by: totalVertices),
                                 indices: indexData.advanced(by: totalIndices),
                                 vertexCount: &vc, indexCount: &ic)
                guard vc > 0, ic > 0 else { continue }
                let isPlane = geom.type == 0
                instanceData[primInstanceIdx] = MJCOffscreenInstanceData(
                    modelMatrix: modelMatrix, normalMatrix: Self.normalMatrix(from: modelMatrix),
                    color: simd_float4(1, 1, 1, 1), emission: geom.emission,
                    specular: geom.specular, shininess: min(max(geom.shininess, 1.0), 10.0),
                    checkerboardScale: isPlane ? 1.0 : 0)
                primDrawList.append((instanceIndex: primInstanceIdx, vertexOffset: totalVertices,
                                     indexOffset: totalIndices, indexCount: ic))
                primInstanceIdx += 1; totalVertices += vc; totalIndices += ic
            }
        }

        // Draw calls
        encoder.setVertexBuffer(instanceBuffer, offset: 0, index: 3)
        encoder.setFragmentBuffer(instanceBuffer, offset: 0, index: 3)

        offset = 0
        for key in activeGroups {
            let count = perGroupCount[key]!
            guard count > 0, let cached = meshCache[key.meshId] else { offset += count; continue }
            let tex = (key.texId >= 0) ? (textureCache[key.texId] ?? placeholderTexture!) : placeholderTexture!
            encoder.setFragmentTexture(tex, index: 0)
            encoder.setFragmentSamplerState(samplerState, index: 0)
            encoder.setVertexBuffer(cached.vertexBuffer, offset: 0, index: 0)
            encoder.drawIndexedPrimitives(
                type: .triangle, indexCount: cached.indexCount, indexType: .uint32,
                indexBuffer: cached.indexBuffer, indexBufferOffset: 0,
                instanceCount: count, baseVertex: 0, baseInstance: offset)
            offset += count
        }

        encoder.setFragmentTexture(placeholderTexture, index: 0)
        encoder.setFragmentSamplerState(samplerState, index: 0)
        for prim in primDrawList {
            encoder.setVertexBuffer(dynamicVertexBuffer,
                                    offset: prim.vertexOffset * MemoryLayout<MJCOffscreenVertex>.stride, index: 0)
            encoder.drawIndexedPrimitives(
                type: .triangle, indexCount: prim.indexCount, indexType: .uint32,
                indexBuffer: dynamicIndexBuffer,
                indexBufferOffset: prim.indexOffset * MemoryLayout<UInt32>.stride,
                instanceCount: 1, baseVertex: 0, baseInstance: prim.instanceIndex)
        }

        encoder.endEncoding()
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()

        return readbackPixels()
    }

    // MARK: - CPU Readback (zero-copy)

    private func readbackPixels() -> Data? {
        guard let buffer = colorBuffer else { return nil }
        // The GPU rendered directly into colorBuffer's linear memory.
        // On Apple Silicon UMA, buffer.contents() points to the same physical
        // pages the GPU wrote to — no copy, no un-tiling, no DMA.
        return Data(bytesNoCopy: buffer.contents(), count: buffer.length, deallocator: .none)
    }

    // MARK: - Matrix Helpers

    static func perspective(fovy: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let ys = 1.0 / tan(fovy * 0.5)
        let xs = ys / aspect
        let zs = far / (near - far)
        return simd_float4x4(columns: (
            simd_float4(xs, 0, 0, 0),
            simd_float4(0, ys, 0, 0),
            simd_float4(0, 0, zs, -1),
            simd_float4(0, 0, near * zs, 0)))
    }

    static func lookAt(eye: simd_float3, center: simd_float3, up: simd_float3) -> simd_float4x4 {
        let f = simd_normalize(center - eye)
        let s = simd_normalize(simd_cross(f, up))
        let u = simd_cross(s, f)
        return simd_float4x4(columns: (
            simd_float4(s.x, u.x, -f.x, 0),
            simd_float4(s.y, u.y, -f.y, 0),
            simd_float4(s.z, u.z, -f.z, 0),
            simd_float4(-simd_dot(s, eye), -simd_dot(u, eye), simd_dot(f, eye), 1)))
    }

    static func normalMatrix(from m: simd_float4x4) -> simd_float4x4 {
        return simd_float4x4(columns: (
            simd_float4(m.columns.0.x, m.columns.0.y, m.columns.0.z, 0),
            simd_float4(m.columns.1.x, m.columns.1.y, m.columns.1.z, 0),
            simd_float4(m.columns.2.x, m.columns.2.y, m.columns.2.z, 0),
            simd_float4(0, 0, 0, 1)))
    }

    // MARK: - Geometry Generators

    private static func convertGeom(geom: MJGeomInstance,
                                     vertices: UnsafeMutablePointer<MJCOffscreenVertex>,
                                     indices: UnsafeMutablePointer<UInt32>,
                                     vertexCount: inout Int, indexCount: inout Int) {
        let color = simd_float4(geom.rgba.0, geom.rgba.1, geom.rgba.2, geom.rgba.3)
        let size = (geom.size.0, geom.size.1, geom.size.2)
        switch geom.type {
        case 0: generatePlane(size: size, color: color, vertices: vertices, indices: indices,
                              vertexCount: &vertexCount, indexCount: &indexCount)
        case 2: generateSphere(radius: geom.size.0, color: color, vertices: vertices, indices: indices,
                               vertexCount: &vertexCount, indexCount: &indexCount)
        case 3: generateCapsule(radius: geom.size.0, halfLength: geom.size.2, color: color,
                                vertices: vertices, indices: indices,
                                vertexCount: &vertexCount, indexCount: &indexCount)
        case 5: generateCylinder(radius: geom.size.0, halfLength: geom.size.2, color: color,
                                 vertices: vertices, indices: indices,
                                 vertexCount: &vertexCount, indexCount: &indexCount)
        case 6: generateBox(size: size, color: color, vertices: vertices, indices: indices,
                            vertexCount: &vertexCount, indexCount: &indexCount)
        default: generateBox(size: (0.02, 0.02, 0.02), color: color, vertices: vertices, indices: indices,
                             vertexCount: &vertexCount, indexCount: &indexCount)
        }
    }

    private static func generatePlane(size: (Float, Float, Float), color: simd_float4,
                                       vertices: UnsafeMutablePointer<MJCOffscreenVertex>,
                                       indices: UnsafeMutablePointer<UInt32>,
                                       vertexCount: inout Int, indexCount: inout Int) {
        let sx = size.0 > 0 ? size.0 : Float(500.0)
        let sy = size.1 > 0 ? size.1 : Float(500.0)
        let n = simd_float3(0, 0, 1)
        vertices[0] = MJCOffscreenVertex(position: simd_float3(-sx, -sy, 0), normal: n, color: color)
        vertices[1] = MJCOffscreenVertex(position: simd_float3( sx, -sy, 0), normal: n, color: color)
        vertices[2] = MJCOffscreenVertex(position: simd_float3( sx,  sy, 0), normal: n, color: color)
        vertices[3] = MJCOffscreenVertex(position: simd_float3(-sx,  sy, 0), normal: n, color: color)
        indices[0] = 0; indices[1] = 1; indices[2] = 2
        indices[3] = 0; indices[4] = 2; indices[5] = 3
        vertexCount = 4; indexCount = 6
    }

    static func generateSphere(radius: Float, color: simd_float4,
                                vertices: UnsafeMutablePointer<MJCOffscreenVertex>,
                                indices: UnsafeMutablePointer<UInt32>,
                                vertexCount: inout Int, indexCount: inout Int) {
        let segments = 16, rings = 12
        var vCount = 0, iCount = 0
        for r in 0...rings {
            let phi = Float.pi * Float(r) / Float(rings)
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = radius * sin(phi) * cos(theta)
                let y = radius * sin(phi) * sin(theta)
                let z = radius * cos(phi)
                let pos = simd_float3(x, y, z)
                vertices[vCount] = MJCOffscreenVertex(position: pos, normal: simd_normalize(pos),
                    texCoord: simd_float2(Float(s) / Float(segments), Float(r) / Float(rings)), color: color)
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
        vertexCount = vCount; indexCount = iCount
    }

    static func generateBox(size: (Float, Float, Float), color: simd_float4,
                             vertices: UnsafeMutablePointer<MJCOffscreenVertex>,
                             indices: UnsafeMutablePointer<UInt32>,
                             vertexCount: inout Int, indexCount: inout Int) {
        let sx = size.0, sy = size.1, sz = size.2
        let corners: [simd_float3] = [
            simd_float3(-sx, -sy, -sz), simd_float3(sx, -sy, -sz),
            simd_float3(sx, sy, -sz), simd_float3(-sx, sy, -sz),
            simd_float3(-sx, -sy, sz), simd_float3(sx, -sy, sz),
            simd_float3(sx, sy, sz), simd_float3(-sx, sy, sz)]
        let faceIndices: [[Int]] = [
            [0,1,2,3], [4,7,6,5], [0,4,5,1], [2,6,7,3], [0,3,7,4], [1,5,6,2]]
        let faceNormals: [simd_float3] = [
            simd_float3(0,0,-1), simd_float3(0,0,1), simd_float3(0,-1,0),
            simd_float3(0,1,0), simd_float3(-1,0,0), simd_float3(1,0,0)]
        var vCount = 0, iCount = 0
        for f in 0..<6 {
            let base = vCount
            for v in 0..<4 {
                vertices[vCount] = MJCOffscreenVertex(
                    position: corners[faceIndices[f][v]], normal: faceNormals[f],
                    texCoord: simd_float2(Float(v % 2), Float(v / 2)), color: color)
                vCount += 1
            }
            indices[iCount] = UInt32(base); iCount += 1
            indices[iCount] = UInt32(base + 1); iCount += 1
            indices[iCount] = UInt32(base + 2); iCount += 1
            indices[iCount] = UInt32(base); iCount += 1
            indices[iCount] = UInt32(base + 2); iCount += 1
            indices[iCount] = UInt32(base + 3); iCount += 1
        }
        vertexCount = vCount; indexCount = iCount
    }

    static func generateCylinder(radius: Float, halfLength: Float, color: simd_float4,
                                  vertices: UnsafeMutablePointer<MJCOffscreenVertex>,
                                  indices: UnsafeMutablePointer<UInt32>,
                                  vertexCount: inout Int, indexCount: inout Int) {
        let segments = 16
        var vCount = 0, iCount = 0
        for s in 0...segments {
            let theta = 2.0 * Float.pi * Float(s) / Float(segments)
            let x = radius * cos(theta), y = radius * sin(theta)
            let n = simd_float3(cos(theta), sin(theta), 0)
            vertices[vCount] = MJCOffscreenVertex(position: simd_float3(x, y, -halfLength), normal: n, color: color)
            vCount += 1
            vertices[vCount] = MJCOffscreenVertex(position: simd_float3(x, y, halfLength), normal: n, color: color)
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
        vertexCount = vCount; indexCount = iCount
    }

    static func generateCapsule(radius: Float, halfLength: Float, color: simd_float4,
                                 vertices: UnsafeMutablePointer<MJCOffscreenVertex>,
                                 indices: UnsafeMutablePointer<UInt32>,
                                 vertexCount: inout Int, indexCount: inout Int) {
        generateCylinder(radius: radius, halfLength: halfLength, color: color,
                         vertices: vertices, indices: indices,
                         vertexCount: &vertexCount, indexCount: &indexCount)
        let segments = 16, rings = 8
        // Bottom hemisphere
        var capBase = vertexCount
        for r in 0...rings {
            let phi = Float.pi * 0.5 + Float.pi * 0.5 * Float(r) / Float(rings)
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = radius * sin(phi) * cos(theta)
                let y = radius * sin(phi) * sin(theta)
                let z = radius * cos(phi) - halfLength
                let n = simd_normalize(simd_float3(x, y, z + halfLength))
                vertices[vertexCount] = MJCOffscreenVertex(position: simd_float3(x, y, z), normal: n, color: color)
                vertexCount += 1
            }
        }
        for r in 0..<rings {
            for s in 0..<segments {
                let curr = capBase + r * (segments + 1) + s
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
        capBase = vertexCount
        for r in 0...rings {
            let phi = Float.pi * 0.5 * Float(r) / Float(rings)
            for s in 0...segments {
                let theta = 2.0 * Float.pi * Float(s) / Float(segments)
                let x = radius * sin(Float.pi * 0.5 - phi) * cos(theta)
                let y = radius * sin(Float.pi * 0.5 - phi) * sin(theta)
                let z = radius * cos(Float.pi * 0.5 - phi) + halfLength
                let n = simd_normalize(simd_float3(x, y, z - halfLength))
                vertices[vertexCount] = MJCOffscreenVertex(position: simd_float3(x, y, z), normal: n, color: color)
                vertexCount += 1
            }
        }
        for r in 0..<rings {
            for s in 0..<segments {
                let curr = capBase + r * (segments + 1) + s
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

public enum MJCOffscreenRenderError: Error, LocalizedError {
    case initFailed(String)

    public var errorDescription: String? {
        switch self {
        case .initFailed(let msg): return "Offscreen render init failed: \(msg)"
        }
    }
}
