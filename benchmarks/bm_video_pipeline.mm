// bm_video_pipeline.mm
// Benchmarks for each stage of the video streaming pipeline.
//
// Run with: bazel run //benchmarks:bm_video_pipeline --config=release
//
// Stages benchmarked:
//   1. CRC-32 checksum computation
//   2. JPEG encode (CoreGraphics CGImageDestination)
//   3. RTP packetize + send (RFC 2435 JPEG over RTP)
//   4. MJPEG HTTP frame send (multipart/x-mixed-replace)
//   5. Raw UDP fragment + send (custom protocol)

#include <benchmark/benchmark.h>

#include "mjc_video_types.h"
#include "mjc_video_rtp_transport.h"
#include "mjc_video_mjpeg_server.h"
#include "mjc_video_udp_transport.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <thread>
#include <vector>

#include <CoreGraphics/CoreGraphics.h>
#include <ImageIO/ImageIO.h>

// ============================================================================
// Helpers
// ============================================================================

// Create synthetic RGBA pixel data (gradient pattern).
static std::vector<uint8_t> MakeSyntheticRGBA(int width, int height) {
    std::vector<uint8_t> rgba(static_cast<size_t>(width * height * 4));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            size_t idx = static_cast<size_t>((y * width + x) * 4);
            rgba[idx + 0] = static_cast<uint8_t>(x);
            rgba[idx + 1] = static_cast<uint8_t>(y);
            rgba[idx + 2] = static_cast<uint8_t>(x + y);
            rgba[idx + 3] = 255;
        }
    }
    return rgba;
}

// Encode RGBA to JPEG using CoreGraphics (pure CoreFoundation API, no ObjC).
static std::vector<uint8_t> EncodeJPEG(const uint8_t* rgba,
                                         int width, int height, float quality) {
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    CGDataProviderRef provider = CGDataProviderCreateWithData(
        nullptr, rgba, static_cast<size_t>(width * height * 4), nullptr);
    CGImageRef image = CGImageCreate(
        width, height, 8, 32, width * 4, cs,
        kCGImageAlphaNoneSkipLast, provider, nullptr, false,
        kCGRenderingIntentDefault);

    CFMutableDataRef jpegData = CFDataCreateMutable(nullptr, 0);
    CGImageDestinationRef dest = CGImageDestinationCreateWithData(
        jpegData, CFSTR("public.jpeg"), 1, nullptr);

    // Compression quality as CFDictionary
    CFNumberRef qualityNum = CFNumberCreate(nullptr, kCFNumberFloatType, &quality);
    CFStringRef key = kCGImageDestinationLossyCompressionQuality;
    CFDictionaryRef opts = CFDictionaryCreate(
        nullptr, (const void**)&key, (const void**)&qualityNum, 1,
        &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

    CGImageDestinationAddImage(dest, image, opts);
    CGImageDestinationFinalize(dest);

    size_t len = static_cast<size_t>(CFDataGetLength(jpegData));
    std::vector<uint8_t> result(len);
    CFDataGetBytes(jpegData, CFRangeMake(0, static_cast<CFIndex>(len)), result.data());

    CFRelease(opts);
    CFRelease(qualityNum);
    CFRelease(dest);
    CFRelease(jpegData);
    CGImageRelease(image);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(cs);
    return result;
}

// Build a MJVideoFrameDesc for benchmarking.
static MJVideoFrameDesc MakeFrameDesc(int width, int height, size_t dataSize,
                                        MJVideoFormat format) {
    MJVideoFrameDesc desc{};
    desc.width = static_cast<uint32_t>(width);
    desc.height = static_cast<uint32_t>(height);
    desc.stride = static_cast<uint32_t>(width * 4);
    desc.format = static_cast<uint8_t>(format);
    desc.camera_index = 0;
    desc.reserved[0] = 0;
    desc.reserved[1] = 0;
    desc.simulation_time = 1.0;
    desc.frame_number = 1;
    desc.data_size = static_cast<uint32_t>(dataSize);
    desc.checksum = 0;
    return desc;
}

// ============================================================================
// 1. CRC-32 Checksum
// ============================================================================

static void BM_CRC32(benchmark::State& state) {
    size_t size = static_cast<size_t>(state.range(0));
    std::vector<uint8_t> data(size, 0x42);

    for (auto _ : state) {
        benchmark::DoNotOptimize(mjc_video_crc32(data.data(), data.size()));
    }
    state.SetBytesProcessed(static_cast<int64_t>(state.iterations() * size));
}

BENCHMARK(BM_CRC32)
    ->Arg(10 * 1024)    // ~10KB (typical small JPEG)
    ->Arg(20 * 1024)    // ~20KB (typical 256x256 JPEG)
    ->Arg(262144)        // 256KB (256x256 raw RGBA)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// 2. JPEG Encode (CoreGraphics)
// ============================================================================

static void BM_JPEGEncode(benchmark::State& state) {
    int width = static_cast<int>(state.range(0));
    int height = width;
    auto rgba = MakeSyntheticRGBA(width, height);

    for (auto _ : state) {
        @autoreleasepool {
            auto jpeg = EncodeJPEG(rgba.data(), width, height, 0.8f);
            benchmark::DoNotOptimize(jpeg.data());
            benchmark::ClobberMemory();
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
    // Report JPEG output size as custom counter
    auto sample = EncodeJPEG(rgba.data(), width, height, 0.8f);
    state.counters["jpeg_KB"] = static_cast<double>(sample.size()) / 1024.0;
    state.counters["pixels"] = static_cast<double>(width * height);
}

BENCHMARK(BM_JPEGEncode)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// 3. RTP SendFrame (JPEG parse + RFC 2435 packetize + sendto)
// ============================================================================

class RTPFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State& state) override {
        int width = static_cast<int>(state.range(0));
        int height = width;

        // Pre-encode synthetic JPEG
        auto rgba = MakeSyntheticRGBA(width, height);
        jpeg_ = EncodeJPEG(rgba.data(), width, height, 0.8f);
        desc_ = MakeFrameDesc(width, height, jpeg_.size(), MJVideoFormat::JPEG);
        desc_.checksum = mjc_video_crc32(jpeg_.data(), jpeg_.size());

        // Create loopback receiver socket (so sendto succeeds)
        recv_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in recv_addr{};
        recv_addr.sin_family = AF_INET;
        recv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        recv_addr.sin_port = htons(0);  // OS picks port
        bind(recv_fd_, reinterpret_cast<struct sockaddr*>(&recv_addr),
             sizeof(recv_addr));

        // Get assigned port
        socklen_t addr_len = sizeof(recv_addr);
        getsockname(recv_fd_, reinterpret_cast<struct sockaddr*>(&recv_addr),
                     &addr_len);

        // Large receive buffer to absorb benchmark traffic
        int rcvbuf = 4 * 1024 * 1024;
        setsockopt(recv_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        // Create and start RTP transport
        rtp_ = MJVideoRTPTransport::create();
        rtp_->Start(ntohs(recv_addr.sin_port) + 10);
        rtp_->AddClient(recv_addr);
    }

    void TearDown(const benchmark::State&) override {
        rtp_->Stop();
        MJVideoRTPTransport::destroy(rtp_);
        close(recv_fd_);
    }

protected:
    MJVideoRTPTransport* rtp_ = nullptr;
    int recv_fd_ = -1;
    std::vector<uint8_t> jpeg_;
    MJVideoFrameDesc desc_{};
};

BENCHMARK_DEFINE_F(RTPFixture, SendFrame)(benchmark::State& state) {
    for (auto _ : state) {
        rtp_->SendFrame(desc_, jpeg_.data(), jpeg_.size());
    }
    state.SetBytesProcessed(
        static_cast<int64_t>(state.iterations() * jpeg_.size()));
    state.counters["jpeg_KB"] = static_cast<double>(jpeg_.size()) / 1024.0;
    // Estimate RTP packet count: jpeg_size / ~1300 payload bytes
    state.counters["rtp_pkts"] = static_cast<double>(
        (jpeg_.size() + 1300 - 1) / 1300);
}

BENCHMARK_REGISTER_F(RTPFixture, SendFrame)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// 4. MJPEG SendJPEG (HTTP multipart send over TCP loopback)
// ============================================================================

class MJPEGFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State& state) override {
        int width = static_cast<int>(state.range(0));
        int height = width;

        auto rgba = MakeSyntheticRGBA(width, height);
        jpeg_ = EncodeJPEG(rgba.data(), width, height, 0.8f);

        // Start MJPEG server — use ports below the OS ephemeral range
        // (macOS ephemeral range is 49152-65535, so 18400+ is safe)
        port_ = static_cast<uint16_t>(18400 + width);
        server_ = MJVideoMJPEGServer::create();
        if (!server_->Start(port_)) {
            server_ok_ = false;
            return;
        }
        server_ok_ = true;

        // Connect TCP client
        client_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        addr.sin_port = htons(port_);

        // Wait briefly for server to start listening
        usleep(10000);
        connect(client_fd_, reinterpret_cast<struct sockaddr*>(&addr),
                sizeof(addr));

        // Server expects an HTTP request before sending response header
        const char* http_req = "GET / HTTP/1.1\r\nHost: localhost\r\n\r\n";
        send(client_fd_, http_req, strlen(http_req), 0);

        // Wait for the HTTP response header, then drain it
        usleep(50000);
        char buf[4096];
        recv(client_fd_, buf, sizeof(buf), 0);

        // Large receive buffer
        int rcvbuf = 4 * 1024 * 1024;
        setsockopt(client_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        // Start drain thread to prevent TCP send blocking
        drain_running_.store(true, std::memory_order_release);
        drain_thread_ = std::thread([this] {
            char dbuf[65536];
            while (drain_running_.load(std::memory_order_acquire)) {
                ssize_t n = recv(client_fd_, dbuf, sizeof(dbuf), 0);
                if (n <= 0) break;
            }
        });
    }

    void TearDown(const benchmark::State&) override {
        drain_running_.store(false, std::memory_order_release);
        if (client_fd_ >= 0) {
            shutdown(client_fd_, SHUT_RDWR);
        }
        if (drain_thread_.joinable()) {
            drain_thread_.join();
        }
        if (client_fd_ >= 0) {
            close(client_fd_);
        }
        if (server_) {
            server_->Stop();
            MJVideoMJPEGServer::destroy(server_);
        }
    }

protected:
    MJVideoMJPEGServer* server_ = nullptr;
    bool server_ok_ = false;
    int client_fd_ = -1;
    uint16_t port_ = 0;
    std::vector<uint8_t> jpeg_;
    std::thread drain_thread_;
    std::atomic<bool> drain_running_{false};
};

BENCHMARK_DEFINE_F(MJPEGFixture, SendJPEG)(benchmark::State& state) {
    if (!server_ok_) {
        state.SkipWithError("MJPEG server failed to start");
        return;
    }
    for (auto _ : state) {
        server_->SendJPEG(jpeg_.data(), jpeg_.size());
    }
    state.SetBytesProcessed(
        static_cast<int64_t>(state.iterations() * jpeg_.size()));
    state.counters["jpeg_KB"] = static_cast<double>(jpeg_.size()) / 1024.0;
}

BENCHMARK_REGISTER_F(MJPEGFixture, SendJPEG)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);

// ============================================================================
// 5. Raw UDP SendFrame (fragment + sendto)
// ============================================================================

class UDPFixture : public benchmark::Fixture {
public:
    void SetUp(const benchmark::State& state) override {
        int width = static_cast<int>(state.range(0));
        int height = width;

        auto rgba = MakeSyntheticRGBA(width, height);
        rgba_ = std::move(rgba);
        desc_ = MakeFrameDesc(width, height, rgba_.size(), MJVideoFormat::RGBA8);
        desc_.checksum = mjc_video_crc32(rgba_.data(), rgba_.size());

        // Use ports below the OS ephemeral range (macOS: 49152-65535)
        uint16_t port = static_cast<uint16_t>(18300 + width);

        // Create and start UDP transport
        udp_ = MJVideoUDPTransport::create();
        if (!udp_->Start(port)) {
            return;
        }

        // Create receiver socket and send "hello" to register
        recv_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in recv_addr{};
        recv_addr.sin_family = AF_INET;
        recv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        recv_addr.sin_port = htons(0);  // OS picks port
        bind(recv_fd_, reinterpret_cast<struct sockaddr*>(&recv_addr),
             sizeof(recv_addr));

        // Large receive buffer
        int rcvbuf = 4 * 1024 * 1024;
        setsockopt(recv_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        // Send hello FROM recv_fd TO transport port
        struct sockaddr_in trans_addr{};
        trans_addr.sin_family = AF_INET;
        trans_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        trans_addr.sin_port = htons(port);
        uint8_t hello = 0x01;
        sendto(recv_fd_, &hello, 1, 0,
               reinterpret_cast<struct sockaddr*>(&trans_addr),
               sizeof(trans_addr));

        // Trigger PollForReceiver by calling SendFrame once
        usleep(1000);
        udp_->SendFrame(desc_, rgba_.data(), rgba_.size());
    }

    void TearDown(const benchmark::State&) override {
        if (udp_) {
            udp_->Stop();
            MJVideoUDPTransport::destroy(udp_);
        }
        if (recv_fd_ >= 0) {
            close(recv_fd_);
        }
    }

protected:
    MJVideoUDPTransport* udp_ = nullptr;
    int recv_fd_ = -1;
    std::vector<uint8_t> rgba_;
    MJVideoFrameDesc desc_{};
};

BENCHMARK_DEFINE_F(UDPFixture, SendFrame)(benchmark::State& state) {
    if (!udp_ || !udp_->IsActive()) {
        state.SkipWithError("UDP transport failed to start");
        return;
    }
    for (auto _ : state) {
        udp_->SendFrame(desc_, rgba_.data(), rgba_.size());
    }
    state.SetBytesProcessed(
        static_cast<int64_t>(state.iterations() * rgba_.size()));
    state.counters["data_KB"] = static_cast<double>(rgba_.size()) / 1024.0;
    // Estimate fragment count
    state.counters["fragments"] = static_cast<double>(
        (sizeof(MJVideoFrameDesc) + rgba_.size() + 1400 - 1) / 1400);
}

BENCHMARK_REGISTER_F(UDPFixture, SendFrame)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
