// validate_pipeline.mm
// Validates that each benchmark stage produces correct output.
// Writes intermediate results to /tmp for visual inspection.
//
// Run: bazel run //benchmarks:validate_pipeline

#import <Metal/Metal.h>
#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>

#include <cstdio>
#include <cstring>
#include <vector>
#include <mach/mach_time.h>

// Same shader as the benchmark
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

struct BenchVertex {
    float position[3];
    float color[4];
};

static double MachToMicroseconds(uint64_t elapsed) {
    static mach_timebase_info_data_t info = {};
    if (info.denom == 0) mach_timebase_info(&info);
    return (double)elapsed * (double)info.numer / (double)info.denom / 1000.0;
}

static bool WritePNG(const uint8_t* rgba, int width, int height, const char* path) {
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    CGDataProviderRef provider = CGDataProviderCreateWithData(
        nullptr, rgba, (size_t)(width * height * 4), nullptr);
    CGImageRef image = CGImageCreate(
        width, height, 8, 32, width * 4, cs,
        kCGImageAlphaNoneSkipLast, provider, nullptr, false,
        kCGRenderingIntentDefault);

    CFURLRef url = CFURLCreateFromFileSystemRepresentation(
        nullptr, (const UInt8*)path, (CFIndex)strlen(path), false);
    CGImageDestinationRef dest = CGImageDestinationCreateWithURL(
        url, CFSTR("public.png"), 1, nullptr);

    CGImageDestinationAddImage(dest, image, nullptr);
    bool ok = CGImageDestinationFinalize(dest);

    CFRelease(dest);
    CFRelease(url);
    CGImageRelease(image);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(cs);
    return ok;
}

static std::vector<uint8_t> EncodeJPEG(const uint8_t* rgba, int width, int height, float quality) {
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    CGDataProviderRef provider = CGDataProviderCreateWithData(
        nullptr, rgba, (size_t)(width * height * 4), nullptr);
    CGImageRef image = CGImageCreate(
        width, height, 8, 32, width * 4, cs,
        kCGImageAlphaNoneSkipLast, provider, nullptr, false,
        kCGRenderingIntentDefault);

    CFMutableDataRef jpegData = CFDataCreateMutable(nullptr, 0);
    CGImageDestinationRef dest = CGImageDestinationCreateWithData(
        jpegData, CFSTR("public.jpeg"), 1, nullptr);

    CFNumberRef qualityNum = CFNumberCreate(nullptr, kCFNumberFloatType, &quality);
    CFStringRef key = kCGImageDestinationLossyCompressionQuality;
    CFDictionaryRef opts = CFDictionaryCreate(
        nullptr, (const void**)&key, (const void**)&qualityNum, 1,
        &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

    CGImageDestinationAddImage(dest, image, opts);
    CGImageDestinationFinalize(dest);

    size_t len = (size_t)CFDataGetLength(jpegData);
    std::vector<uint8_t> result(len);
    CFDataGetBytes(jpegData, CFRangeMake(0, (CFIndex)len), result.data());

    CFRelease(opts);
    CFRelease(qualityNum);
    CFRelease(dest);
    CFRelease(jpegData);
    CGImageRelease(image);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(cs);
    return result;
}

int main() {
    @autoreleasepool {
        const int W = 256, H = 256;
        printf("=== Pipeline Validation (%dx%d) ===\n\n", W, H);

        // ---- 1. Create Metal device ----
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (!device) { printf("FAIL: No Metal device\n"); return 1; }
        printf("Device: %s\n", [[device name] UTF8String]);

        id<MTLCommandQueue> queue = [device newCommandQueue];

        // Compile shader
        NSError* error = nil;
        id<MTLLibrary> library = [device newLibraryWithSource:
            [NSString stringWithUTF8String:kShaderSource] options:nil error:&error];
        if (!library) {
            printf("FAIL: Shader compile: %s\n", [[error localizedDescription] UTF8String]);
            return 1;
        }

        id<MTLFunction> vertexFunc = [library newFunctionWithName:@"benchVertexMain"];
        id<MTLFunction> fragmentFunc = [library newFunctionWithName:@"benchFragmentMain"];

        // Vertex descriptor
        MTLVertexDescriptor* vertexDesc = [[MTLVertexDescriptor alloc] init];
        vertexDesc.attributes[0].format = MTLVertexFormatFloat3;
        vertexDesc.attributes[0].offset = 0;
        vertexDesc.attributes[0].bufferIndex = 0;
        vertexDesc.attributes[1].format = MTLVertexFormatFloat4;
        vertexDesc.attributes[1].offset = sizeof(float) * 3;
        vertexDesc.attributes[1].bufferIndex = 0;
        vertexDesc.layouts[0].stride = sizeof(BenchVertex);

        MTLRenderPipelineDescriptor* pipelineDesc = [[MTLRenderPipelineDescriptor alloc] init];
        pipelineDesc.vertexFunction = vertexFunc;
        pipelineDesc.fragmentFunction = fragmentFunc;
        pipelineDesc.vertexDescriptor = vertexDesc;
        pipelineDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA8Unorm;
        pipelineDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;

        id<MTLRenderPipelineState> pipeline =
            [device newRenderPipelineStateWithDescriptor:pipelineDesc error:&error];
        if (!pipeline) { printf("FAIL: Pipeline state\n"); return 1; }

        MTLDepthStencilDescriptor* depthDesc = [[MTLDepthStencilDescriptor alloc] init];
        depthDesc.depthCompareFunction = MTLCompareFunctionLess;
        depthDesc.depthWriteEnabled = YES;
        id<MTLDepthStencilState> depthState = [device newDepthStencilStateWithDescriptor:depthDesc];

        // ---- 2. Create textures (buffer-backed + regular) ----
        NSUInteger bytesPerRow = (NSUInteger)(W * 4);
        NSUInteger bufferSize = bytesPerRow * (NSUInteger)H;

        // Buffer-backed (zero-copy)
        id<MTLBuffer> colorBuffer = [device newBufferWithLength:bufferSize
                                                        options:MTLResourceStorageModeShared];
        MTLTextureDescriptor* colorDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                         width:W height:H mipmapped:NO];
        colorDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        colorDesc.storageMode = MTLStorageModeShared;
        id<MTLTexture> bufferTexture = [colorBuffer newTextureWithDescriptor:colorDesc
                                                                      offset:0
                                                                 bytesPerRow:bytesPerRow];

        // Regular texture (getBytes path)
        id<MTLTexture> regularTexture = [device newTextureWithDescriptor:colorDesc];

        // Depth
        MTLTextureDescriptor* depthTexDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                         width:W height:H mipmapped:NO];
        depthTexDesc.usage = MTLTextureUsageRenderTarget;
        depthTexDesc.storageMode = MTLStorageModePrivate;
        id<MTLTexture> depthTexture = [device newTextureWithDescriptor:depthTexDesc];

        // ---- 3. Create geometry (colorful triangles) ----
        const int triCount = 500;
        std::vector<BenchVertex> vertices(triCount * 3);
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
        id<MTLBuffer> vertexBuffer = [device newBufferWithBytes:vertices.data()
                                                         length:vertices.size() * sizeof(BenchVertex)
                                                        options:MTLResourceStorageModeShared];

        printf("\n--- Stage 1: Metal Render (500 triangles) ---\n");

        // Lambda to render to a given texture
        auto renderTo = [&](id<MTLTexture> target) {
            id<MTLCommandBuffer> cmdBuf = [queue commandBuffer];
            MTLRenderPassDescriptor* passDesc = [MTLRenderPassDescriptor renderPassDescriptor];
            passDesc.colorAttachments[0].texture = target;
            passDesc.colorAttachments[0].loadAction = MTLLoadActionClear;
            passDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
            passDesc.colorAttachments[0].clearColor = MTLClearColorMake(0.15, 0.18, 0.22, 1.0);
            passDesc.depthAttachment.texture = depthTexture;
            passDesc.depthAttachment.loadAction = MTLLoadActionClear;
            passDesc.depthAttachment.storeAction = MTLStoreActionDontCare;
            passDesc.depthAttachment.clearDepth = 1.0;

            id<MTLRenderCommandEncoder> enc = [cmdBuf renderCommandEncoderWithDescriptor:passDesc];
            [enc setRenderPipelineState:pipeline];
            [enc setDepthStencilState:depthState];
            [enc setVertexBuffer:vertexBuffer offset:0 atIndex:0];
            [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:triCount * 3];
            [enc endEncoding];

            uint64_t t0 = mach_absolute_time();
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
            uint64_t t1 = mach_absolute_time();
            return MachToMicroseconds(t1 - t0);
        };

        // Warm up (first command buffer is always slow)
        renderTo(bufferTexture);

        // Timed render to buffer-backed texture
        double renderTime = renderTo(bufferTexture);
        printf("  Render + waitUntilCompleted: %.1f us\n", renderTime);

        // ---- 4. Validate buffer-backed readback ----
        printf("\n--- Stage 2: Zero-copy Readback ---\n");
        const uint8_t* pixels = (const uint8_t*)[colorBuffer contents];

        // Check pixel statistics
        int nonZeroPixels = 0;
        int uniqueColors = 0;
        uint8_t prevR = 0, prevG = 0, prevB = 0;
        size_t totalPixels = (size_t)(W * H);
        for (size_t i = 0; i < totalPixels; i++) {
            uint8_t r = pixels[i * 4 + 0];
            uint8_t g = pixels[i * 4 + 1];
            uint8_t b = pixels[i * 4 + 2];
            if (r != 0 || g != 0 || b != 0) nonZeroPixels++;
            if (r != prevR || g != prevG || b != prevB) {
                uniqueColors++;
                prevR = r; prevG = g; prevB = b;
            }
        }
        printf("  Non-zero pixels: %d / %zu (%.1f%%)\n",
               nonZeroPixels, totalPixels, 100.0 * nonZeroPixels / totalPixels);
        printf("  Unique color transitions: %d\n", uniqueColors);

        // Check clear color (0.15, 0.18, 0.22) = (38, 46, 56) in RGBA8
        // Sample a corner pixel (should be clear color)
        printf("  Corner pixel [0,0]: RGBA(%u, %u, %u, %u)\n",
               pixels[0], pixels[1], pixels[2], pixels[3]);
        // Sample center pixel (might have a triangle)
        size_t centerIdx = (size_t)(H / 2 * W + W / 2) * 4;
        printf("  Center pixel [%d,%d]: RGBA(%u, %u, %u, %u)\n",
               W / 2, H / 2,
               pixels[centerIdx], pixels[centerIdx + 1],
               pixels[centerIdx + 2], pixels[centerIdx + 3]);

        // Write PNG for visual inspection
        WritePNG(pixels, W, H, "/tmp/pipeline_validate_buffer.png");
        printf("  Wrote: /tmp/pipeline_validate_buffer.png\n");

        // ---- 5. Compare with getBytes readback ----
        printf("\n--- Stage 3: getBytes Readback (comparison) ---\n");
        renderTo(regularTexture);

        std::vector<uint8_t> getBytesPixels(bufferSize);
        uint64_t t0 = mach_absolute_time();
        [regularTexture getBytes:getBytesPixels.data()
                     bytesPerRow:bytesPerRow
                      fromRegion:MTLRegionMake2D(0, 0, W, H)
                     mipmapLevel:0];
        uint64_t t1 = mach_absolute_time();
        printf("  getBytes time: %.1f us\n", MachToMicroseconds(t1 - t0));

        WritePNG(getBytesPixels.data(), W, H, "/tmp/pipeline_validate_getbytes.png");
        printf("  Wrote: /tmp/pipeline_validate_getbytes.png\n");

        // Compare the two
        int diffPixels = 0;
        for (size_t i = 0; i < bufferSize; i++) {
            if (pixels[i] != getBytesPixels[i]) diffPixels++;
        }
        printf("  Byte differences vs buffer-backed: %d / %zu\n", diffPixels, bufferSize);

        // ---- 6. JPEG encode ----
        printf("\n--- Stage 4: JPEG Encode ---\n");
        t0 = mach_absolute_time();
        auto jpeg = EncodeJPEG(pixels, W, H, 0.8f);
        t1 = mach_absolute_time();
        printf("  JPEG encode time: %.1f us\n", MachToMicroseconds(t1 - t0));
        printf("  JPEG size: %zu bytes (%.1f KB)\n", jpeg.size(), jpeg.size() / 1024.0);

        // Validate JPEG header
        bool validJPEG = jpeg.size() > 2 && jpeg[0] == 0xFF && jpeg[1] == 0xD8;
        printf("  JPEG header (FFD8): %s\n", validJPEG ? "OK" : "FAIL");
        // Check JPEG footer
        bool validFooter = jpeg.size() > 2 &&
            jpeg[jpeg.size() - 2] == 0xFF && jpeg[jpeg.size() - 1] == 0xD9;
        printf("  JPEG footer (FFD9): %s\n", validFooter ? "OK" : "FAIL");

        // Write JPEG for inspection
        FILE* f = fopen("/tmp/pipeline_validate.jpg", "wb");
        if (f) {
            fwrite(jpeg.data(), 1, jpeg.size(), f);
            fclose(f);
            printf("  Wrote: /tmp/pipeline_validate.jpg\n");
        }

        // ---- 7. Run multiple iterations to check consistency ----
        printf("\n--- Stage 5: Timing Consistency (10 iterations) ---\n");
        printf("  %4s  %10s  %10s  %10s\n", "#", "Render(us)", "JPEG(us)", "Total(us)");
        for (int iter = 0; iter < 10; iter++) {
            uint64_t t_render_start = mach_absolute_time();
            renderTo(bufferTexture);
            uint64_t t_render_end = mach_absolute_time();

            const uint8_t* px = (const uint8_t*)[colorBuffer contents];

            uint64_t t_jpeg_start = mach_absolute_time();
            auto j = EncodeJPEG(px, W, H, 0.8f);
            uint64_t t_jpeg_end = mach_absolute_time();

            double render_us = MachToMicroseconds(t_render_end - t_render_start);
            double jpeg_us = MachToMicroseconds(t_jpeg_end - t_jpeg_start);
            printf("  %4d  %10.1f  %10.1f  %10.1f\n",
                   iter, render_us, jpeg_us, render_us + jpeg_us);
        }

        printf("\n=== Validation complete ===\n");
        printf("Inspect images:\n");
        printf("  open /tmp/pipeline_validate_buffer.png\n");
        printf("  open /tmp/pipeline_validate_getbytes.png\n");
        printf("  open /tmp/pipeline_validate.jpg\n");
    }
    return 0;
}
