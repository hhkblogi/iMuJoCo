// bm_fragment.cc
// Benchmark for UDP fragmentation and reassembly round-trip

#include <benchmark/benchmark.h>

#include "fragment.h"

#include <cstdint>
#include <numeric>
#include <vector>

using namespace imujoco::driver;

// Benchmark: Fragment a message of given size
static void BM_FragmentMessage(benchmark::State& state) {
    const size_t msg_size = static_cast<size_t>(state.range(0));
    std::vector<uint8_t> data(msg_size);
    std::iota(data.begin(), data.end(), static_cast<uint8_t>(0));

    FragmentedSender sender;
    for (auto _ : state) {
        auto fragments = sender.FragmentMessage(data.data(), data.size());
        benchmark::DoNotOptimize(fragments);
    }
    state.SetBytesProcessed(state.iterations() * msg_size);
    state.counters["fragments"] = static_cast<double>(
        (msg_size + kMaxFragmentPayload - 1) / kMaxFragmentPayload);
}

BENCHMARK(BM_FragmentMessage)
    ->Arg(256)       // Small: single fragment
    ->Arg(1400)      // Near MTU: single fragment
    ->Arg(4096)      // 3 fragments
    ->Arg(16384)     // 12 fragments
    ->Arg(65536)     // 45 fragments
    ->Arg(262144)    // 180 fragments (typical large state)
    ->Unit(benchmark::kMicrosecond);

// Benchmark: Reassemble fragments back into a complete message
static void BM_ReassembleMessage(benchmark::State& state) {
    const size_t msg_size = static_cast<size_t>(state.range(0));
    std::vector<uint8_t> data(msg_size);
    std::iota(data.begin(), data.end(), static_cast<uint8_t>(0));

    // Pre-fragment the message
    FragmentedSender sender;
    auto fragments = sender.FragmentMessage(data.data(), data.size());

    ReassemblyManager reassembler;
    for (auto _ : state) {
        reassembler.Reset();
        ReassemblyResult result;
        for (const auto& frag : fragments) {
            result = reassembler.ProcessFragment(frag.data(), frag.size());
        }
        benchmark::DoNotOptimize(result);
    }
    state.SetBytesProcessed(state.iterations() * msg_size);
}

BENCHMARK(BM_ReassembleMessage)
    ->Arg(256)
    ->Arg(1400)
    ->Arg(4096)
    ->Arg(16384)
    ->Arg(65536)
    ->Arg(262144)
    ->Unit(benchmark::kMicrosecond);

// Benchmark: Full round-trip (fragment + reassemble)
static void BM_FragmentRoundTrip(benchmark::State& state) {
    const size_t msg_size = static_cast<size_t>(state.range(0));
    std::vector<uint8_t> data(msg_size);
    std::iota(data.begin(), data.end(), static_cast<uint8_t>(0));

    FragmentedSender sender;
    ReassemblyManager reassembler;

    for (auto _ : state) {
        auto fragments = sender.FragmentMessage(data.data(), data.size());
        reassembler.Reset();
        ReassemblyResult result;
        for (const auto& frag : fragments) {
            result = reassembler.ProcessFragment(frag.data(), frag.size());
        }
        benchmark::DoNotOptimize(result);
    }
    state.SetBytesProcessed(state.iterations() * msg_size);
}

BENCHMARK(BM_FragmentRoundTrip)
    ->Arg(256)
    ->Arg(4096)
    ->Arg(65536)
    ->Arg(262144)
    ->Unit(benchmark::kMicrosecond);

// Benchmark: CRC-16 computation
static void BM_CRC16(benchmark::State& state) {
    const size_t size = static_cast<size_t>(state.range(0));
    std::vector<uint8_t> data(size);
    std::iota(data.begin(), data.end(), static_cast<uint8_t>(0));

    for (auto _ : state) {
        auto crc = ComputeCRC16(data.data(), data.size());
        benchmark::DoNotOptimize(crc);
    }
    state.SetBytesProcessed(state.iterations() * size);
}

BENCHMARK(BM_CRC16)
    ->Arg(16)     // Fragment header
    ->Arg(1456)   // Max fragment payload
    ->Unit(benchmark::kNanosecond);

BENCHMARK_MAIN();
