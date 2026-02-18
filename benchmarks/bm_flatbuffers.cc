// bm_flatbuffers.cc
// Benchmark for FlatBuffers state/control packet serialization

#include <benchmark/benchmark.h>

#include "control_generated.h"
#include "state_generated.h"

#include <cstdint>
#include <vector>

using namespace imujoco::schema;

// Helper: create a StatePacketT with given DOF counts
static StatePacketT MakeState(int nq, int nv, int nu, int nsensor) {
    StatePacketT s;
    s.sequence = 42;
    s.time = 1.234;
    s.energy_potential = -9.81;
    s.energy_kinetic = 3.14;
    s.qpos.resize(nq, 0.1);
    s.qvel.resize(nv, 0.2);
    s.ctrl.resize(nu, 0.3);
    s.sensordata.resize(nsensor, 0.4);
    return s;
}

// Benchmark: Serialize a StatePacket (Pack)
static void BM_StatePacketSerialize(benchmark::State& state) {
    const int nq = static_cast<int>(state.range(0));
    const int nv = static_cast<int>(state.range(1));
    const int nu = static_cast<int>(state.range(2));
    const int nsensor = static_cast<int>(state.range(3));

    auto st = MakeState(nq, nv, nu, nsensor);
    flatbuffers::FlatBufferBuilder builder(4096);

    for (auto _ : state) {
        builder.Clear();
        auto offset = StatePacket::Pack(builder, &st);
        FinishStatePacketBuffer(builder, offset);
        benchmark::DoNotOptimize(builder.GetBufferPointer());
    }
    state.SetBytesProcessed(state.iterations() * builder.GetSize());
    state.counters["buf_bytes"] = static_cast<double>(builder.GetSize());
}

BENCHMARK(BM_StatePacketSerialize)
    // {nq, nv, nu, nsensor}
    ->Args({7, 6, 1, 0})        // Simple: cart-pole
    ->Args({29, 27, 21, 100})   // Medium: humanoid
    ->Args({51, 50, 37, 200})   // Large: G1 robot
    ->Args({200, 150, 100, 500})// XL: multi-robot
    ->Unit(benchmark::kMicrosecond);

// Benchmark: Deserialize a StatePacket (UnPack)
static void BM_StatePacketDeserialize(benchmark::State& state) {
    const int nq = static_cast<int>(state.range(0));
    const int nv = static_cast<int>(state.range(1));
    const int nu = static_cast<int>(state.range(2));
    const int nsensor = static_cast<int>(state.range(3));

    auto st = MakeState(nq, nv, nu, nsensor);
    flatbuffers::FlatBufferBuilder builder(4096);
    auto offset = StatePacket::Pack(builder, &st);
    FinishStatePacketBuffer(builder, offset);

    const uint8_t* buf = builder.GetBufferPointer();
    size_t buf_size = builder.GetSize();

    for (auto _ : state) {
        auto packet = GetStatePacket(buf);
        StatePacketT unpacked;
        packet->UnPackTo(&unpacked);
        benchmark::DoNotOptimize(unpacked);
    }
    state.SetBytesProcessed(state.iterations() * buf_size);
}

BENCHMARK(BM_StatePacketDeserialize)
    ->Args({7, 6, 1, 0})
    ->Args({29, 27, 21, 100})
    ->Args({51, 50, 37, 200})
    ->Args({200, 150, 100, 500})
    ->Unit(benchmark::kMicrosecond);

// Benchmark: Verify a StatePacket buffer
static void BM_StatePacketVerify(benchmark::State& state) {
    const int nq = static_cast<int>(state.range(0));
    const int nv = static_cast<int>(state.range(1));

    auto st = MakeState(nq, nv, nq, 0);
    flatbuffers::FlatBufferBuilder builder(4096);
    auto offset = StatePacket::Pack(builder, &st);
    FinishStatePacketBuffer(builder, offset);

    const uint8_t* buf = builder.GetBufferPointer();
    size_t buf_size = builder.GetSize();

    for (auto _ : state) {
        flatbuffers::Verifier verifier(buf, buf_size);
        bool ok = VerifyStatePacketBuffer(verifier);
        benchmark::DoNotOptimize(ok);
    }
    state.SetBytesProcessed(state.iterations() * buf_size);
}

BENCHMARK(BM_StatePacketVerify)
    ->Args({7, 6})
    ->Args({29, 27})
    ->Args({51, 50})
    ->Args({200, 150})
    ->Unit(benchmark::kNanosecond);

// Benchmark: Serialize a ControlPacket
static void BM_ControlPacketSerialize(benchmark::State& state) {
    const int nu = static_cast<int>(state.range(0));

    ControlPacketT cmd;
    cmd.sequence = 1;
    cmd.host_timestamp_us = 12345678;
    cmd.ctrl.resize(nu, 0.5);

    flatbuffers::FlatBufferBuilder builder(256);

    for (auto _ : state) {
        builder.Clear();
        auto offset = ControlPacket::Pack(builder, &cmd);
        FinishControlPacketBuffer(builder, offset);
        benchmark::DoNotOptimize(builder.GetBufferPointer());
    }
    state.SetBytesProcessed(state.iterations() * builder.GetSize());
}

BENCHMARK(BM_ControlPacketSerialize)
    ->Arg(1)    // Single actuator
    ->Arg(21)   // Humanoid
    ->Arg(37)   // G1 robot
    ->Arg(100)  // Multi-robot
    ->Unit(benchmark::kNanosecond);

BENCHMARK_MAIN();
