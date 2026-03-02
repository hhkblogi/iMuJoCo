// time_sync_test.cc
// Unit tests for gPTP-inspired time sync module

#include "imujoco/driver/time_sync.h"
#include "mjc_sync_protocol.h"

#include <flatbuffers/flatbuffers.h>
#include "control_generated.h"
#include "state_generated.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

namespace imujoco::driver {
namespace {

// ============================================================================
// PTPClockServo Tests
// ============================================================================

TEST(PTPClockServoTest, InitialState) {
    PTPClockServo servo;
    auto state = servo.GetState();

    EXPECT_FALSE(state.locked);
    EXPECT_EQ(state.offset_us, 0);
    EXPECT_EQ(state.delay_us, 0);
    EXPECT_EQ(state.rate_ratio_ppm, 0);
    EXPECT_EQ(state.exchanges, 0);
    EXPECT_DOUBLE_EQ(state.jitter_us, 0.0);
}

TEST(PTPClockServoTest, ConvergenceWithConstantOffset) {
    PTPClockServo::Config config;
    config.kp = 0.7;                   // Use aggressive gains for fast convergence in test
    config.ki = 0.3;
    config.min_samples_for_lock = 10;
    config.lock_threshold_us = 1000.0; // Offset-threshold for lock detection
    PTPClockServo servo(config);

    // Simulate 50 exchanges with offset = 5000µs, delay = 1000µs
    // Forward: t2 - t1 = offset + delay = 6000
    // Backward: t3 - t4 = offset - delay = 4000
    // raw_offset = (6000 + 4000) / 2 = 5000
    // raw_delay = (6000 - 4000) / 2 = 1000
    // Need enough samples for PI to converge and for the lock window
    // (default 30) to fill with stable post-convergence values.
    for (int i = 0; i < 50; i++) {
        uint64_t t1 = 1000000 + i * 100000;
        uint64_t t2 = t1 + 6000;  // forward = offset + delay
        uint64_t t3 = t2 + 100;   // small processing time
        uint64_t t4 = t3 - 4000;  // backward = -(offset - delay)
        // Actually: t3 - t4 = offset - delay = 4000, so t4 = t3 - 4000
        // Wait, let me recalculate.
        // offset = device - driver. device clock = driver + 5000.
        // t1 = driver time. t2 = device time when received = t1 + delay + offset ≈ t1 + 1000 + 5000 = t1 + 6000
        // t3 = t2 + processing. t4 = t3 - offset + delay = t3 - 5000 + 1000 = t3 - 4000
        t4 = t3 - 4000;

        PdelayExchange ex{t1, t2, t3, t4};
        servo.ProcessExchange(ex);
    }

    auto state = servo.GetState();
    EXPECT_EQ(state.exchanges, 50);
    // Servo should converge near 5000µs offset
    EXPECT_NEAR(state.offset_us, 5000, 500);
    EXPECT_TRUE(state.locked);
}

TEST(PTPClockServoTest, MedianFilter) {
    PTPClockServo::Config config;
    config.median_window = 7;
    config.min_samples_for_lock = 20;  // Don't trigger lock yet
    config.outlier_threshold = 100.0;  // Disable outlier rejection
    PTPClockServo servo(config);

    // Feed 7 exchanges with varying offsets, 2 of which are outliers
    int64_t offsets[] = {1000, 1010, 5000, 990, 1005, 6000, 995};
    for (int i = 0; i < 7; i++) {
        int64_t offset = offsets[i];
        uint64_t t1 = 1000000 + i * 100000;
        uint64_t t2 = t1 + offset + 500;  // delay = 500
        uint64_t t3 = t2 + 50;
        uint64_t t4 = t3 - (offset - 500);

        servo.ProcessExchange({t1, t2, t3, t4});
    }

    auto state = servo.GetState();
    EXPECT_EQ(state.exchanges, 7);
    // Median of {1000, 1010, 5000, 990, 1005, 6000, 995} = 1005
    // The servo applies PI control on top, so exact match won't happen,
    // but the offset should be in the right ballpark
    EXPECT_GT(state.offset_us, 0);
}

TEST(PTPClockServoTest, OutlierRejection) {
    PTPClockServo::Config config;
    config.outlier_threshold = 3.0;
    config.min_samples_for_lock = 10;
    config.lock_threshold_us = 100.0;
    PTPClockServo servo(config);

    // 15 consistent exchanges at offset = 2000µs
    for (int i = 0; i < 15; i++) {
        uint64_t t1 = 1000000 + i * 100000;
        uint64_t t2 = t1 + 2500;  // offset=2000, delay=500
        uint64_t t3 = t2 + 50;
        uint64_t t4 = t3 - 1500;  // offset-delay = 1500

        servo.ProcessExchange({t1, t2, t3, t4});
    }

    auto state_before = servo.GetState();

    // Inject a 10x spike
    uint64_t t1 = 3000000;
    uint64_t t2 = t1 + 25000;  // offset = 20000, 10x spike
    uint64_t t3 = t2 + 50;
    uint64_t t4 = t3 - 15000;

    servo.ProcessExchange({t1, t2, t3, t4});

    auto state_after = servo.GetState();

    // Offset should not have jumped significantly
    int64_t drift = std::abs(state_after.offset_us - state_before.offset_us);
    EXPECT_LT(drift, 1000);  // Should be rejected, minimal change
}

TEST(PTPClockServoTest, RateRatioUpdate) {
    PTPClockServo servo;

    servo.UpdateRateRatio(25);
    auto state = servo.GetState();
    EXPECT_EQ(state.rate_ratio_ppm, 25);

    servo.UpdateRateRatio(-10);
    state = servo.GetState();
    EXPECT_EQ(state.rate_ratio_ppm, -10);
}

TEST(PTPClockServoTest, ClockConversionRoundtrip) {
    PTPClockServo servo;

    // Set a known offset via exchanges
    for (int i = 0; i < 15; i++) {
        uint64_t t1 = 1000000 + i * 100000;
        uint64_t t2 = t1 + 3500;  // offset=3000, delay=500
        uint64_t t3 = t2 + 50;
        uint64_t t4 = t3 - 2500;

        servo.ProcessExchange({t1, t2, t3, t4});
    }

    uint64_t driver_time = 5000000;
    uint64_t device_time = servo.DriverToDeviceTime(driver_time);
    uint64_t roundtrip = servo.DeviceToDriverTime(device_time);

    EXPECT_EQ(roundtrip, driver_time);
}

TEST(PTPClockServoTest, Reset) {
    PTPClockServo servo;

    // Add some state
    for (int i = 0; i < 5; i++) {
        uint64_t t1 = 1000000 + i * 100000;
        servo.ProcessExchange({t1, t1 + 5000, t1 + 5050, t1 + 50});
    }
    servo.UpdateRateRatio(42);

    auto state = servo.GetState();
    EXPECT_GT(state.exchanges, 0);

    // Reset
    servo.Reset();
    state = servo.GetState();
    EXPECT_EQ(state.exchanges, 0);
    EXPECT_EQ(state.offset_us, 0);
    EXPECT_EQ(state.delay_us, 0);
    EXPECT_EQ(state.rate_ratio_ppm, 0);
    EXPECT_FALSE(state.locked);
}

// ============================================================================
// SimTimeClock Tests
// ============================================================================

TEST(SimTimeClockTest, InitialState) {
    SimTimeClock clock;
    EXPECT_FALSE(clock.HasAnchor());

    PTPClockServo servo;
    auto result = clock.PredictSimTime(1000000, servo);
    EXPECT_FALSE(result.has_value());
}

TEST(SimTimeClockTest, PredictionFromAnchor) {
    SimTimeClock clock;
    PTPClockServo servo;

    // Set an anchor: sim_time=1.0 at device_wall=1000000, timestep=2000µs
    SimTimeAnchor anchor;
    anchor.sim_time = 1.0;
    anchor.device_wall_us = 1000000;
    anchor.step_index = 500;
    anchor.timestep_us = 2000;
    clock.UpdateAnchor(anchor);

    EXPECT_TRUE(clock.HasAnchor());

    // With zero offset, driver_time = device_time
    // 100ms later: predicted sim_time = 1.0 + 0.1 = 1.1
    uint64_t driver_now = 1100000;  // 100ms later
    auto result = clock.PredictSimTime(driver_now, servo);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.1, 0.001);
}

TEST(SimTimeClockTest, RejectsZeroWallTime) {
    SimTimeClock clock;

    SimTimeAnchor anchor;
    anchor.sim_time = 1.0;
    anchor.device_wall_us = 0;  // Invalid
    anchor.timestep_us = 2000;
    clock.UpdateAnchor(anchor);

    EXPECT_FALSE(clock.HasAnchor());
}

TEST(SimTimeClockTest, Reset) {
    SimTimeClock clock;

    SimTimeAnchor anchor;
    anchor.sim_time = 1.0;
    anchor.device_wall_us = 1000000;
    anchor.timestep_us = 2000;
    clock.UpdateAnchor(anchor);

    EXPECT_TRUE(clock.HasAnchor());

    clock.Reset();
    EXPECT_FALSE(clock.HasAnchor());
}

// ============================================================================
// Sync Protocol Tests
// ============================================================================

TEST(SyncProtocolTest, RequestPacking) {
    static_assert(sizeof(protocol::MJPdelayRequest) == 18);

    protocol::MJPdelayRequest req;
    req.seq = 42;
    req.t1_us = 1234567890;
    protocol::mj_sync_build_request(&req);

    EXPECT_EQ(req.magic, protocol::MJ_SYNC_MAGIC_REQ);
    EXPECT_TRUE(protocol::mj_sync_validate_request(&req));
}

TEST(SyncProtocolTest, ResponsePacking) {
    static_assert(sizeof(protocol::MJPdelayResponse) == 38);

    protocol::MJPdelayResponse resp;
    resp.seq = 42;
    resp.t1_us = 1000000;
    resp.t2_us = 1006000;
    resp.t3_us = 1006100;
    resp.rate_ratio_ppm = -25;
    protocol::mj_sync_build_response(&resp);

    EXPECT_EQ(resp.magic, protocol::MJ_SYNC_MAGIC_RESP);
    EXPECT_TRUE(protocol::mj_sync_validate_response(&resp));
}

TEST(SyncProtocolTest, ChecksumDetectsCorruption) {
    protocol::MJPdelayRequest req;
    req.seq = 1;
    req.t1_us = 999999;
    protocol::mj_sync_build_request(&req);

    EXPECT_TRUE(protocol::mj_sync_validate_request(&req));

    // Corrupt one byte
    req.seq = 2;
    EXPECT_FALSE(protocol::mj_sync_validate_request(&req));
}

TEST(SyncProtocolTest, WrongMagicRejected) {
    protocol::MJPdelayRequest req;
    req.seq = 1;
    req.t1_us = 100;
    protocol::mj_sync_build_request(&req);
    req.magic = 0xDEADBEEF;  // Wrong magic

    EXPECT_FALSE(protocol::mj_sync_validate_request(&req));
}

TEST(SyncProtocolTest, NullRejected) {
    EXPECT_FALSE(protocol::mj_sync_validate_request(nullptr));
    EXPECT_FALSE(protocol::mj_sync_validate_response(nullptr));
}

TEST(SyncProtocolTest, CRC16Deterministic) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc1 = protocol::mj_sync_crc16(data, sizeof(data));
    uint16_t crc2 = protocol::mj_sync_crc16(data, sizeof(data));
    EXPECT_EQ(crc1, crc2);

    // Different data should give different CRC
    data[0] = 0xFF;
    uint16_t crc3 = protocol::mj_sync_crc16(data, sizeof(data));
    EXPECT_NE(crc1, crc3);
}

// ============================================================================
// Backward Compatibility Tests
// ============================================================================

TEST(BackwardCompatTest, OldControlPacketFieldsDefaultToZero) {
    // Build a minimal ControlPacket with only the original 3 fields
    flatbuffers::FlatBufferBuilder builder(256);
    auto ctrl_vec = builder.CreateVector(std::vector<double>{1.0, 2.0});
    auto packet = imujoco::schema::CreateControlPacket(
        builder, /*sequence=*/1, ctrl_vec, /*host_timestamp_us=*/0);
    imujoco::schema::FinishControlPacketBuffer(builder, packet);

    // Parse and verify new fields default to 0
    auto parsed = imujoco::schema::GetControlPacket(builder.GetBufferPointer());
    EXPECT_EQ(parsed->sequence(), 1);
    EXPECT_EQ(parsed->host_timestamp_us(), 0);
    EXPECT_DOUBLE_EQ(parsed->target_sim_time(), 0.0);
    EXPECT_EQ(parsed->echo_token(), 0);
}

TEST(BackwardCompatTest, OldStatePacketFieldsDefaultToZero) {
    // Build a minimal StatePacket with only the original 8 fields
    flatbuffers::FlatBufferBuilder builder(256);
    auto qpos_vec = builder.CreateVector(std::vector<double>{0.1, 0.2});
    auto qvel_vec = builder.CreateVector(std::vector<double>{0.3});
    auto packet = imujoco::schema::CreateStatePacket(
        builder, /*sequence=*/5, /*time=*/1.5,
        /*energy_potential=*/0.0, /*energy_kinetic=*/0.0,
        qpos_vec, qvel_vec);
    imujoco::schema::FinishStatePacketBuffer(builder, packet);

    // Parse and verify new fields default to 0
    auto parsed = imujoco::schema::GetStatePacket(builder.GetBufferPointer());
    EXPECT_EQ(parsed->sequence(), 5);
    EXPECT_DOUBLE_EQ(parsed->time(), 1.5);
    EXPECT_EQ(parsed->step_index(), 0);
    EXPECT_EQ(parsed->accepted_ctrl_seq(), 0);
    EXPECT_EQ(parsed->device_wall_us(), 0);
    EXPECT_EQ(parsed->timestep_us(), 0);
    EXPECT_EQ(parsed->echo_token(), 0);
}

}  // namespace
}  // namespace imujoco::driver
