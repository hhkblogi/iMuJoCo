// bm_metal_render.mm
// Benchmarks for Metal GPU rendering and readback stages.
//
// Run with: bazel run //benchmarks:bm_metal_render --config=release
//
// Stages benchmarked:
//   1. Clear-only render pass + waitUntilCompleted (GPU-CPU sync overhead)
//   2. Render synthetic triangles + waitUntilCompleted (GPU render time)
//   3. Buffer-backed zero-copy readback (new approach)
//   4. getBytes readback from regular texture (old approach)

#import <Metal/Metal.h>

#include <benchmark/benchmark.h>
#include <vector>

// ============================================================================
// Shader source (compiled at runtime)
// ============================================================================

static const char* kShaderSource = R"(
#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position [[attribute(0)]];
    float4 color    [[attribute(1)]];
};

struct VertexOut {
    float4 position [[position]];
    float4 color;
};

vertex VertexOut benchVertexMain(VertexIn in [[stage_in]]) {
    VertexOut out;
    out.position = float4(in.position, 1.0);
    out.color = in.color;
    return out;
}

fragment float4 benchFragmentMain(VertexOut in [[stage_in]]) {
    return in.color;
}
)";

// ============================================================================
// Shared Metal state (created once per fixture)
// ============================================================================

struct MetalState {
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;
    id<MTLRenderPipelineState> pipeline;
    id<MTLDepthStencilState> depthState;

    // Buffer-backed color texture (zero-copy)
    id<MTLBuffer> colorBuffer;
    id<MTLTexture> colorTexture;
    id<MTLTexture> depthTexture;

    // Regular texture (getBytes path)
    id<MTLTexture> regularTexture;

    // Geometry
    id<MTLBuffer> vertexBuffer;
    int vertexCount;

    int width;
    int height;
};

// Vertex layout: float3 position + float4 color = 28 bytes
struct BenchVertex {
    float position[3];
    float color[4];
};

static bool CreateMetalState(MetalState& state, int width, int height, int triCount) {
    state.width = width;
    state.height = height;

    state.device = MTLCreateSystemDefaultDevice();
    if (!state.device) return false;

    state.queue = [state.device newCommandQueue];
    if (!state.queue) return false;

    // Compile shaders from source
    NSError* error = nil;
    id<MTLLibrary> library = [state.device newLibraryWithSource:
        [NSString stringWithUTF8String:kShaderSource] options:nil error:&error];
    if (!library) return false;

    id<MTLFunction> vertexFunc = [library newFunctionWithName:@"benchVertexMain"];
    id<MTLFunction> fragmentFunc = [library newFunctionWithName:@"benchFragmentMain"];
    if (!vertexFunc || !fragmentFunc) return false;

    // Vertex descriptor
    MTLVertexDescriptor* vertexDesc = [[MTLVertexDescriptor alloc] init];
    vertexDesc.attributes[0].format = MTLVertexFormatFloat3;
    vertexDesc.attributes[0].offset = 0;
    vertexDesc.attributes[0].bufferIndex = 0;
    vertexDesc.attributes[1].format = MTLVertexFormatFloat4;
    vertexDesc.attributes[1].offset = sizeof(float) * 3;
    vertexDesc.attributes[1].bufferIndex = 0;
    vertexDesc.layouts[0].stride = sizeof(BenchVertex);
    vertexDesc.layouts[0].stepFunction = MTLVertexStepFunctionPerVertex;

    // Pipeline state
    MTLRenderPipelineDescriptor* pipelineDesc = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineDesc.vertexFunction = vertexFunc;
    pipelineDesc.fragmentFunction = fragmentFunc;
    pipelineDesc.vertexDescriptor = vertexDesc;
    pipelineDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA8Unorm;
    pipelineDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;

    state.pipeline = [state.device newRenderPipelineStateWithDescriptor:pipelineDesc error:&error];
    if (!state.pipeline) return false;

    // Depth state
    MTLDepthStencilDescriptor* depthDesc = [[MTLDepthStencilDescriptor alloc] init];
    depthDesc.depthCompareFunction = MTLCompareFunctionLess;
    depthDesc.depthWriteEnabled = YES;
    state.depthState = [state.device newDepthStencilStateWithDescriptor:depthDesc];

    // Buffer-backed color texture (zero-copy approach)
    NSUInteger bytesPerRow = static_cast<NSUInteger>(width * 4);
    NSUInteger bufferSize = bytesPerRow * static_cast<NSUInteger>(height);
    state.colorBuffer = [state.device newBufferWithLength:bufferSize
                                                 options:MTLResourceStorageModeShared];
    if (!state.colorBuffer) return false;

    MTLTextureDescriptor* colorDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:static_cast<NSUInteger>(width)
                                    height:static_cast<NSUInteger>(height)
                                 mipmapped:NO];
    colorDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    colorDesc.storageMode = MTLStorageModeShared;
    state.colorTexture = [state.colorBuffer newTextureWithDescriptor:colorDesc
                                                              offset:0
                                                         bytesPerRow:bytesPerRow];
    if (!state.colorTexture) return false;

    // Regular texture (getBytes approach — for comparison)
    MTLTextureDescriptor* regDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:static_cast<NSUInteger>(width)
                                    height:static_cast<NSUInteger>(height)
                                 mipmapped:NO];
    regDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    regDesc.storageMode = MTLStorageModeShared;
    state.regularTexture = [state.device newTextureWithDescriptor:regDesc];

    // Depth texture
    MTLTextureDescriptor* depthTexDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                     width:static_cast<NSUInteger>(width)
                                    height:static_cast<NSUInteger>(height)
                                 mipmapped:NO];
    depthTexDesc.usage = MTLTextureUsageRenderTarget;
    depthTexDesc.storageMode = MTLStorageModePrivate;
    state.depthTexture = [state.device newTextureWithDescriptor:depthTexDesc];

    // Generate synthetic triangles (random-ish positions filling the viewport)
    state.vertexCount = triCount * 3;
    std::vector<BenchVertex> vertices(static_cast<size_t>(state.vertexCount));
    for (int t = 0; t < triCount; t++) {
        float cx = (float)(t % 32) / 16.0f - 1.0f;
        float cy = (float)(t / 32) / 16.0f - 1.0f;
        float r = 0.03f;
        float cr = (float)(t * 37 % 256) / 255.0f;
        float cg = (float)(t * 73 % 256) / 255.0f;
        float cb = (float)(t * 113 % 256) / 255.0f;
        vertices[t * 3 + 0] = {{cx - r, cy - r, 0.5f}, {cr, cg, cb, 1.0f}};
        vertices[t * 3 + 1] = {{cx + r, cy - r, 0.5f}, {cr, cg, cb, 1.0f}};
        vertices[t * 3 + 2] = {{cx, cy + r, 0.5f}, {cr, cg, cb, 1.0f}};
    }
    state.vertexBuffer = [state.device newBufferWithBytes:vertices.data()
                                                  length:vertices.size() * sizeof(BenchVertex)
                                                 options:MTLResourceStorageModeShared];

    return true;
}

// Helper: encode a render pass (clear + optional draw)
static void EncodeRenderPass(const MetalState& state,
                             id<MTLCommandBuffer> cmdBuf,
                             id<MTLTexture> colorTarget,
                             bool draw) {
    MTLRenderPassDescriptor* passDesc = [MTLRenderPassDescriptor renderPassDescriptor];
    passDesc.colorAttachments[0].texture = colorTarget;
    passDesc.colorAttachments[0].loadAction = MTLLoadActionClear;
    passDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
    passDesc.colorAttachments[0].clearColor = MTLClearColorMake(0.15, 0.18, 0.22, 1.0);
    passDesc.depthAttachment.texture = state.depthTexture;
    passDesc.depthAttachment.loadAction = MTLLoadActionClear;
    passDesc.depthAttachment.storeAction = MTLStoreActionDontCare;
    passDesc.depthAttachment.clearDepth = 1.0;

    id<MTLRenderCommandEncoder> encoder = [cmdBuf renderCommandEncoderWithDescriptor:passDesc];
    if (draw) {
        [encoder setRenderPipelineState:state.pipeline];
        [encoder setDepthStencilState:state.depthState];
        [encoder setVertexBuffer:state.vertexBuffer offset:0 atIndex:0];
        [encoder drawPrimitives:MTLPrimitiveTypeTriangle
                    vertexStart:0
                    vertexCount:static_cast<NSUInteger>(state.vertexCount)];
    }
    [encoder endEncoding];
}

// ============================================================================
// 1. Clear-only render pass (GPU-CPU sync overhead)
// ============================================================================

class MetalClearFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State& state) override {
        int width = static_cast<int>(state.range(0));
        ok_ = CreateMetalState(state_, width, width, 0);
    }
    void TearDown(const benchmark::State&) override {}

protected:
    MetalState state_{};
    bool ok_ = false;
};

BENCHMARK_DEFINE_F(MetalClearFixture, ClearOnly)(benchmark::State& state) {
    if (!ok_) { state.SkipWithError("Metal init failed"); return; }
    for (auto _ : state) {
        @autoreleasepool {
            id<MTLCommandBuffer> cmdBuf = [state_.queue commandBuffer];
            EncodeRenderPass(state_, cmdBuf, state_.colorTexture, false);
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
    state.counters["pixels"] = static_cast<double>(state_.width * state_.height);
}

BENCHMARK_REGISTER_F(MetalClearFixture, ClearOnly)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// 2. Render synthetic triangles (GPU render + sync)
// ============================================================================

class MetalRenderFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State& state) override {
        int width = static_cast<int>(state.range(0));
        int triCount = static_cast<int>(state.range(1));
        ok_ = CreateMetalState(state_, width, width, triCount);
    }
    void TearDown(const benchmark::State&) override {}

protected:
    MetalState state_{};
    bool ok_ = false;
};

BENCHMARK_DEFINE_F(MetalRenderFixture, DrawTriangles)(benchmark::State& state) {
    if (!ok_) { state.SkipWithError("Metal init failed"); return; }
    for (auto _ : state) {
        @autoreleasepool {
            id<MTLCommandBuffer> cmdBuf = [state_.queue commandBuffer];
            EncodeRenderPass(state_, cmdBuf, state_.colorTexture, true);
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
    state.counters["pixels"] = static_cast<double>(state_.width * state_.height);
    state.counters["triangles"] = static_cast<double>(state.range(1));
}

BENCHMARK_REGISTER_F(MetalRenderFixture, DrawTriangles)
    ->Args({256, 100})      // 100 triangles (simple scene)
    ->Args({256, 500})      // 500 triangles (typical MuJoCo)
    ->Args({256, 2000})     // 2000 triangles (complex scene)
    ->Args({512, 500})      // 512x512 with 500 triangles
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// 3. Buffer-backed readback (zero-copy on UMA)
// ============================================================================

class MetalReadbackFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State& state) override {
        int width = static_cast<int>(state.range(0));
        ok_ = CreateMetalState(state_, width, width, 100);
        if (!ok_) return;

        // Pre-render a frame so there's valid data in the texture
        @autoreleasepool {
            id<MTLCommandBuffer> cmdBuf = [state_.queue commandBuffer];
            EncodeRenderPass(state_, cmdBuf, state_.colorTexture, true);
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
        }
        // Also render to the regular texture
        @autoreleasepool {
            id<MTLCommandBuffer> cmdBuf = [state_.queue commandBuffer];
            EncodeRenderPass(state_, cmdBuf, state_.regularTexture, true);
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
        }
        readbackBuf_.resize(static_cast<size_t>(width * width * 4));
    }
    void TearDown(const benchmark::State&) override {}

protected:
    MetalState state_{};
    bool ok_ = false;
    std::vector<uint8_t> readbackBuf_;
};

// Zero-copy: just access buffer.contents() — no copy on UMA
BENCHMARK_DEFINE_F(MetalReadbackFixture, ZeroCopy)(benchmark::State& state) {
    if (!ok_) { state.SkipWithError("Metal init failed"); return; }
    for (auto _ : state) {
        const void* ptr = [state_.colorBuffer contents];
        benchmark::DoNotOptimize(ptr);
        // Simulate what the JPEG encoder does: read first + last byte
        const uint8_t* bytes = static_cast<const uint8_t*>(ptr);
        uint8_t first = bytes[0];
        uint8_t last = bytes[state_.colorBuffer.length - 1];
        benchmark::DoNotOptimize(first);
        benchmark::DoNotOptimize(last);
        benchmark::ClobberMemory();
    }
    state.SetBytesProcessed(
        static_cast<int64_t>(state.iterations() * state_.width * state_.height * 4));
}

BENCHMARK_REGISTER_F(MetalReadbackFixture, ZeroCopy)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kNanosecond);

// getBytes: copies from texture's internal storage to CPU buffer
BENCHMARK_DEFINE_F(MetalReadbackFixture, GetBytes)(benchmark::State& state) {
    if (!ok_) { state.SkipWithError("Metal init failed"); return; }
    NSUInteger bytesPerRow = static_cast<NSUInteger>(state_.width * 4);
    MTLRegion region = MTLRegionMake2D(0, 0,
        static_cast<NSUInteger>(state_.width),
        static_cast<NSUInteger>(state_.height));
    for (auto _ : state) {
        [state_.regularTexture getBytes:readbackBuf_.data()
                            bytesPerRow:bytesPerRow
                             fromRegion:region
                            mipmapLevel:0];
        benchmark::DoNotOptimize(readbackBuf_.data());
        benchmark::ClobberMemory();
    }
    state.SetBytesProcessed(
        static_cast<int64_t>(state.iterations() * state_.width * state_.height * 4));
}

BENCHMARK_REGISTER_F(MetalReadbackFixture, GetBytes)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// 4. Full pipeline: render + readback (combined)
// ============================================================================

class MetalFullPipelineFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State& state) override {
        int width = static_cast<int>(state.range(0));
        ok_ = CreateMetalState(state_, width, width, 500);
    }
    void TearDown(const benchmark::State&) override {}

protected:
    MetalState state_{};
    bool ok_ = false;
};

BENCHMARK_DEFINE_F(MetalFullPipelineFixture, RenderAndReadback)(benchmark::State& state) {
    if (!ok_) { state.SkipWithError("Metal init failed"); return; }
    for (auto _ : state) {
        @autoreleasepool {
            id<MTLCommandBuffer> cmdBuf = [state_.queue commandBuffer];
            EncodeRenderPass(state_, cmdBuf, state_.colorTexture, true);
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
            // Zero-copy readback
            const void* ptr = [state_.colorBuffer contents];
            benchmark::DoNotOptimize(ptr);
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
    state.counters["pixels"] = static_cast<double>(state_.width * state_.height);
}

BENCHMARK_REGISTER_F(MetalFullPipelineFixture, RenderAndReadback)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
