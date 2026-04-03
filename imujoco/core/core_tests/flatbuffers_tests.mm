// flatbuffers_tests.mm
// Tests for FlatBuffers schema serialization/deserialization

#import <XCTest/XCTest.h>

#include "imujoco/schema/state_generated.h"
#include "imujoco/schema/control_generated.h"

using namespace imujoco::schema;

@interface FlatBuffersTests : XCTestCase
@end

@implementation FlatBuffersTests

#pragma mark - StatePacket Tests

- (void)test_state_packet_creation {
    flatbuffers::FlatBufferBuilder builder(256);

    // Create test data
    std::vector<double> qpos = {1.0, 2.0, 3.0};
    std::vector<double> qvel = {0.1, 0.2, 0.3};
    std::vector<double> ctrl = {0.5, 0.6};
    std::vector<double> sensordata = {100.0, 200.0, 300.0, 400.0};

    auto qpos_vec = builder.CreateVector(qpos);
    auto qvel_vec = builder.CreateVector(qvel);
    auto ctrl_vec = builder.CreateVector(ctrl);
    auto sensor_vec = builder.CreateVector(sensordata);

    auto state = CreateStatePacket(builder,
        42,           // sequence
        1.5,          // time
        10.0,         // energy_potential
        5.0,          // energy_kinetic
        qpos_vec,
        qvel_vec,
        ctrl_vec,
        sensor_vec
    );

    builder.Finish(state);

    // Verify buffer was created
    XCTAssertTrue(builder.GetSize() > 0, @"Buffer should have content");
}

- (void)test_state_packet_round_trip {
    flatbuffers::FlatBufferBuilder builder(256);

    // Create test data
    std::vector<double> qpos = {1.0, 2.0, 3.0, 4.0};
    std::vector<double> qvel = {0.1, 0.2, 0.3, 0.4};
    std::vector<double> ctrl = {0.5, 0.6};
    std::vector<double> sensordata = {100.0, 200.0};

    auto qpos_vec = builder.CreateVector(qpos);
    auto qvel_vec = builder.CreateVector(qvel);
    auto ctrl_vec = builder.CreateVector(ctrl);
    auto sensor_vec = builder.CreateVector(sensordata);

    uint32_t sequence = 12345;
    double time = 2.5;
    double energy_potential = 15.5;
    double energy_kinetic = 8.3;

    auto state = CreateStatePacket(builder,
        sequence,
        time,
        energy_potential,
        energy_kinetic,
        qpos_vec,
        qvel_vec,
        ctrl_vec,
        sensor_vec
    );

    builder.Finish(state);

    // Read back the data
    auto buf = builder.GetBufferPointer();
    auto packet = GetStatePacket(buf);

    // Verify scalar fields
    XCTAssertEqual(packet->sequence(), sequence);
    XCTAssertEqualWithAccuracy(packet->time(), time, 0.0001);
    XCTAssertEqualWithAccuracy(packet->energy_potential(), energy_potential, 0.0001);
    XCTAssertEqualWithAccuracy(packet->energy_kinetic(), energy_kinetic, 0.0001);

    // Verify vector fields
    XCTAssertEqual(packet->qpos()->size(), static_cast<unsigned int>(qpos.size()));
    XCTAssertEqual(packet->qvel()->size(), static_cast<unsigned int>(qvel.size()));
    XCTAssertEqual(packet->ctrl()->size(), static_cast<unsigned int>(ctrl.size()));
    XCTAssertEqual(packet->sensordata()->size(), static_cast<unsigned int>(sensordata.size()));

    // Verify vector contents
    for (size_t i = 0; i < qpos.size(); i++) {
        XCTAssertEqualWithAccuracy(packet->qpos()->Get(static_cast<unsigned int>(i)), qpos[i], 0.0001);
    }
    for (size_t i = 0; i < qvel.size(); i++) {
        XCTAssertEqualWithAccuracy(packet->qvel()->Get(static_cast<unsigned int>(i)), qvel[i], 0.0001);
    }
}

- (void)test_state_packet_file_identifier {
    flatbuffers::FlatBufferBuilder builder(256);

    // Create empty vectors (safer than passing 0)
    auto empty_vec = builder.CreateVector(std::vector<double>{});

    auto state = CreateStatePacket(builder, 1, 0.0, 0.0, 0.0,
                                   empty_vec, empty_vec, empty_vec, empty_vec);
    builder.Finish(state, StatePacketIdentifier());

    auto buf = builder.GetBufferPointer();

    // Verify file identifier
    XCTAssertTrue(flatbuffers::BufferHasIdentifier(buf, StatePacketIdentifier()),
                  @"Buffer should have STPK identifier");
}

#pragma mark - ControlPacket Tests

- (void)test_control_packet_creation {
    flatbuffers::FlatBufferBuilder builder(128);

    std::vector<double> ctrl = {0.1, 0.2, 0.3, 0.4, 0.5};
    auto ctrl_vec = builder.CreateVector(ctrl);

    auto control = CreateControlPacket(builder, 99, ctrl_vec);
    builder.Finish(control);

    XCTAssertTrue(builder.GetSize() > 0, @"Buffer should have content");
}

- (void)test_control_packet_round_trip {
    flatbuffers::FlatBufferBuilder builder(128);

    uint32_t sequence = 54321;
    std::vector<double> ctrl = {-1.0, 0.0, 1.0, 0.5, -0.5};
    auto ctrl_vec = builder.CreateVector(ctrl);

    auto control = CreateControlPacket(builder, sequence, ctrl_vec);
    builder.Finish(control);

    // Read back
    auto buf = builder.GetBufferPointer();
    auto packet = GetControlPacket(buf);

    XCTAssertEqual(packet->sequence(), sequence);
    XCTAssertEqual(packet->ctrl()->size(), static_cast<unsigned int>(ctrl.size()));

    for (size_t i = 0; i < ctrl.size(); i++) {
        XCTAssertEqualWithAccuracy(packet->ctrl()->Get(static_cast<unsigned int>(i)), ctrl[i], 0.0001);
    }
}

- (void)test_control_packet_file_identifier {
    flatbuffers::FlatBufferBuilder builder(128);

    // Create empty vector (safer than passing 0)
    auto empty_vec = builder.CreateVector(std::vector<double>{});

    auto control = CreateControlPacket(builder, 1, empty_vec);
    builder.Finish(control, ControlPacketIdentifier());

    auto buf = builder.GetBufferPointer();

    // Verify file identifier
    XCTAssertTrue(flatbuffers::BufferHasIdentifier(buf, ControlPacketIdentifier()),
                  @"Buffer should have CTPK identifier");
}

#pragma mark - Buffer Verification Tests

- (void)test_state_packet_verification {
    flatbuffers::FlatBufferBuilder builder(256);

    // Create all vector fields (verifier requires proper offsets, not 0)
    std::vector<double> qpos = {1.0, 2.0};
    std::vector<double> qvel = {0.1, 0.2};
    std::vector<double> ctrl = {0.5};
    std::vector<double> sensordata = {100.0};

    auto qpos_vec = builder.CreateVector(qpos);
    auto qvel_vec = builder.CreateVector(qvel);
    auto ctrl_vec = builder.CreateVector(ctrl);
    auto sensor_vec = builder.CreateVector(sensordata);

    auto state = CreateStatePacket(builder, 1, 0.0, 0.0, 0.0,
                                   qpos_vec, qvel_vec, ctrl_vec, sensor_vec);
    // Must include file identifier for VerifyStatePacketBuffer to pass
    builder.Finish(state, StatePacketIdentifier());

    auto buf = builder.GetBufferPointer();
    auto size = builder.GetSize();

    // Verify the buffer is valid
    flatbuffers::Verifier verifier(buf, size);
    XCTAssertTrue(VerifyStatePacketBuffer(verifier), @"Buffer should verify successfully");
}

- (void)test_control_packet_verification {
    flatbuffers::FlatBufferBuilder builder(128);

    std::vector<double> ctrl = {0.5};
    auto ctrl_vec = builder.CreateVector(ctrl);

    auto control = CreateControlPacket(builder, 1, ctrl_vec);
    // Must include file identifier for VerifyControlPacketBuffer to pass
    builder.Finish(control, ControlPacketIdentifier());

    auto buf = builder.GetBufferPointer();
    auto size = builder.GetSize();

    // Verify the buffer is valid
    flatbuffers::Verifier verifier(buf, size);
    XCTAssertTrue(VerifyControlPacketBuffer(verifier), @"Buffer should verify successfully");
}

#pragma mark - Extended ControlPacket Tests

- (void)test_control_packet_extended_fields {
    flatbuffers::FlatBufferBuilder builder(512);

    std::vector<double> ctrl = {1.0, 2.0, 3.0};
    std::vector<double> qfrc = {10.0, 20.0};
    std::vector<double> xfrc = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    std::vector<double> mpos = {1.0, 2.0, 3.0};
    std::vector<double> mquat = {1.0, 0.0, 0.0, 0.0};

    // Use Object API to avoid positional arg issues with intervening fields
    ControlPacketT obj;
    obj.sequence = 42;
    obj.ctrl = ctrl;
    obj.host_timestamp_us = 123456;
    obj.qfrc_applied = qfrc;
    obj.xfrc_applied = xfrc;
    obj.mocap_pos = mpos;
    obj.mocap_quat = mquat;
    obj.expire_at_sim_time = 2.5;
    obj.expiry_policy = ExpiryPolicy::HoldLastValid;

    auto control = ControlPacket::Pack(builder, &obj);
    builder.Finish(control, ControlPacketIdentifier());

    auto buf = builder.GetBufferPointer();
    auto packet = GetControlPacket(buf);

    // Verify all fields
    XCTAssertEqual(packet->sequence(), 42u);
    XCTAssertEqual(packet->host_timestamp_us(), 123456ull);
    XCTAssertEqualWithAccuracy(packet->expire_at_sim_time(), 2.5, 0.0001);
    XCTAssertEqual(packet->expiry_policy(), ExpiryPolicy::HoldLastValid);

    XCTAssertEqual(packet->ctrl()->size(), 3u);
    XCTAssertEqualWithAccuracy(packet->ctrl()->Get(0), 1.0, 0.0001);

    XCTAssertEqual(packet->qfrc_applied()->size(), 2u);
    XCTAssertEqualWithAccuracy(packet->qfrc_applied()->Get(0), 10.0, 0.0001);

    XCTAssertEqual(packet->xfrc_applied()->size(), 6u);
    XCTAssertEqual(packet->mocap_pos()->size(), 3u);
    XCTAssertEqual(packet->mocap_quat()->size(), 4u);
    XCTAssertEqualWithAccuracy(packet->mocap_quat()->Get(0), 1.0, 0.0001);

    // Verify buffer integrity
    flatbuffers::Verifier verifier(buf, builder.GetSize());
    XCTAssertTrue(VerifyControlPacketBuffer(verifier), @"Extended buffer should verify");
}

- (void)test_control_packet_backward_compat {
    // Old-style packet with only ctrl — new fields should be null/default
    flatbuffers::FlatBufferBuilder builder(128);

    std::vector<double> ctrl = {0.5, 0.6};
    auto ctrl_vec = builder.CreateVector(ctrl);

    auto control = CreateControlPacket(builder, 1, ctrl_vec);
    builder.Finish(control, ControlPacketIdentifier());

    auto buf = builder.GetBufferPointer();
    auto packet = GetControlPacket(buf);

    XCTAssertEqual(packet->ctrl()->size(), 2u);
    XCTAssertTrue(packet->qfrc_applied() == nullptr, @"Should be null for old-style packet");
    XCTAssertTrue(packet->xfrc_applied() == nullptr, @"Should be null for old-style packet");
    XCTAssertTrue(packet->mocap_pos() == nullptr, @"Should be null for old-style packet");
    XCTAssertTrue(packet->mocap_quat() == nullptr, @"Should be null for old-style packet");
    XCTAssertEqualWithAccuracy(packet->expire_at_sim_time(), -1.0, 0.0001);
    XCTAssertEqual(packet->expiry_policy(), ExpiryPolicy::Default);
}

- (void)test_expiry_policy_enum {
    // Verify all enum values serialize correctly
    flatbuffers::FlatBufferBuilder builder(128);

    auto policies = {ExpiryPolicy::Default, ExpiryPolicy::ZeroImmediate,
                     ExpiryPolicy::ZeroAfterTimeout, ExpiryPolicy::HoldLastValid};

    for (auto policy : policies) {
        builder.Clear();
        ControlPacketBuilder cp_builder(builder);
        cp_builder.add_expire_at_sim_time(-1.0);
        cp_builder.add_expiry_policy(policy);
        auto control = cp_builder.Finish();
        builder.Finish(control);

        auto packet = GetControlPacket(builder.GetBufferPointer());
        XCTAssertEqual(packet->expiry_policy(), policy,
            @"ExpiryPolicy %d should round-trip", static_cast<int>(policy));
    }
}

@end
