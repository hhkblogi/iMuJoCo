// driver_test.cc
// Unit tests for MuJoCo Driver

#include "imujoco/driver/driver.h"
#include "state_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace imujoco::driver {
namespace {

// Test driver configuration defaults
TEST(DriverTest, ConfigDefaults) {
    DriverConfig config;

    EXPECT_EQ(config.host, "127.0.0.1");
    EXPECT_EQ(config.port, 9000);
    EXPECT_EQ(config.local_port, 0);
    EXPECT_EQ(config.timeout_ms, 100);
    EXPECT_TRUE(config.auto_start_receiving);
}

// Test driver construction and destruction
TEST(DriverTest, Construction) {
    DriverConfig config;
    config.port = 9999;

    Driver driver(config);
    EXPECT_EQ(driver.Config().port, 9999);
    EXPECT_FALSE(driver.IsConnected());
}

// Test statistics initial values
TEST(DriverTest, InitialStats) {
    Driver driver;
    auto stats = driver.GetStats();

    EXPECT_EQ(stats.packets_sent, 0);
    EXPECT_EQ(stats.packets_received, 0);
    EXPECT_EQ(stats.send_errors, 0);
    EXPECT_EQ(stats.receive_errors, 0);
}

// Test statistics reset
TEST(DriverTest, ResetStats) {
    Driver driver;
    driver.ResetStats();

    auto stats = driver.GetStats();
    EXPECT_EQ(stats.packets_sent, 0);
    EXPECT_EQ(stats.packets_received, 0);
}

// Test SimulationState construction
TEST(DriverTest, SimulationState) {
    SimulationState state;

    EXPECT_EQ(state.sequence, 0);
    EXPECT_DOUBLE_EQ(state.time, 0.0);
    EXPECT_DOUBLE_EQ(state.energy_potential, 0.0);
    EXPECT_DOUBLE_EQ(state.energy_kinetic, 0.0);
    EXPECT_TRUE(state.qpos.empty());
    EXPECT_TRUE(state.qvel.empty());
    EXPECT_TRUE(state.ctrl.empty());
    EXPECT_TRUE(state.sensordata.empty());
}

// Test ControlCommand construction
TEST(DriverTest, ControlCommand) {
    ControlCommand cmd;

    EXPECT_EQ(cmd.sequence, 0);
    EXPECT_TRUE(cmd.ctrl.empty());

    // Set some control values
    cmd.ctrl = {1.0, 2.0, 3.0};
    EXPECT_EQ(cmd.ctrl.size(), 3);
}

// Test ControlCommand extended fields and defaults
TEST(DriverTest, ControlCommandExtendedFields) {
    ControlCommand cmd;

    // Verify new fields exist and have correct defaults
    EXPECT_TRUE(cmd.qfrc_applied.empty());
    EXPECT_TRUE(cmd.xfrc_applied.empty());
    EXPECT_TRUE(cmd.mocap_pos.empty());
    EXPECT_TRUE(cmd.mocap_quat.empty());
    EXPECT_DOUBLE_EQ(cmd.expire_at_sim_time, -1.0);
    EXPECT_EQ(cmd.expiry_policy, schema::ExpiryPolicy::Default);

    // Verify fields can be set
    cmd.qfrc_applied = {10.0, 20.0, 30.0};
    cmd.xfrc_applied = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    cmd.mocap_pos = {0.1, 0.2, 0.3};
    cmd.mocap_quat = {1.0, 0.0, 0.0, 0.0};
    cmd.expire_at_sim_time = 1.5;
    cmd.expiry_policy = schema::ExpiryPolicy::ZeroImmediate;

    EXPECT_EQ(cmd.qfrc_applied.size(), 3);
    EXPECT_EQ(cmd.xfrc_applied.size(), 6);
    EXPECT_EQ(cmd.mocap_pos.size(), 3);
    EXPECT_EQ(cmd.mocap_quat.size(), 4);
    EXPECT_DOUBLE_EQ(cmd.expire_at_sim_time, 1.5);
    EXPECT_EQ(cmd.expiry_policy, schema::ExpiryPolicy::ZeroImmediate);
}

// Test ControlCommand FlatBuffers round-trip with extended fields
TEST(DriverTest, ControlCommandRoundTrip) {
    ControlCommand cmd;
    cmd.sequence = 42;
    cmd.ctrl = {1.0, 2.0, 3.0};
    cmd.qfrc_applied = {10.0, 20.0};
    cmd.xfrc_applied = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    cmd.mocap_pos = {1.0, 2.0, 3.0};
    cmd.mocap_quat = {1.0, 0.0, 0.0, 0.0};
    cmd.expire_at_sim_time = 2.5;
    cmd.expiry_policy = schema::ExpiryPolicy::HoldLastValid;
    cmd.host_timestamp_us = 123456;

    // Serialize
    flatbuffers::FlatBufferBuilder builder(256);
    auto offset = schema::ControlPacket::Pack(builder, &cmd);
    schema::FinishControlPacketBuffer(builder, offset);

    // Deserialize
    auto packet = schema::GetControlPacket(builder.GetBufferPointer());
    EXPECT_EQ(packet->sequence(), 42);
    EXPECT_EQ(packet->ctrl()->size(), 3);
    EXPECT_DOUBLE_EQ(packet->ctrl()->Get(0), 1.0);

    ASSERT_NE(packet->qfrc_applied(), nullptr);
    EXPECT_EQ(packet->qfrc_applied()->size(), 2);
    EXPECT_DOUBLE_EQ(packet->qfrc_applied()->Get(0), 10.0);

    ASSERT_NE(packet->xfrc_applied(), nullptr);
    EXPECT_EQ(packet->xfrc_applied()->size(), 6);

    ASSERT_NE(packet->mocap_pos(), nullptr);
    EXPECT_EQ(packet->mocap_pos()->size(), 3);

    ASSERT_NE(packet->mocap_quat(), nullptr);
    EXPECT_EQ(packet->mocap_quat()->size(), 4);
    EXPECT_DOUBLE_EQ(packet->mocap_quat()->Get(0), 1.0);

    EXPECT_DOUBLE_EQ(packet->expire_at_sim_time(), 2.5);
    EXPECT_EQ(packet->expiry_policy(), schema::ExpiryPolicy::HoldLastValid);
    EXPECT_EQ(packet->host_timestamp_us(), 123456);
}

// Test backward compatibility: old-style packet with only ctrl
TEST(DriverTest, ControlCommandBackwardCompat) {
    ControlCommand cmd;
    cmd.ctrl = {1.0, 2.0};

    // Serialize with only ctrl set (other fields empty/default)
    flatbuffers::FlatBufferBuilder builder(256);
    auto offset = schema::ControlPacket::Pack(builder, &cmd);
    schema::FinishControlPacketBuffer(builder, offset);

    // Deserialize and verify defaults
    auto packet = schema::GetControlPacket(builder.GetBufferPointer());
    EXPECT_EQ(packet->ctrl()->size(), 2);
    EXPECT_EQ(packet->qfrc_applied(), nullptr);
    EXPECT_EQ(packet->xfrc_applied(), nullptr);
    EXPECT_EQ(packet->mocap_pos(), nullptr);
    EXPECT_EQ(packet->mocap_quat(), nullptr);
    EXPECT_DOUBLE_EQ(packet->expire_at_sim_time(), -1.0);
    EXPECT_EQ(packet->expiry_policy(), schema::ExpiryPolicy::Default);
}

// Test subscriber management
TEST(DriverTest, SubscriberManagement) {
    Driver driver;

    std::atomic<int> callback_count{0};

    // Subscribe
    auto id1 = driver.Subscribe([&](const SimulationState&) {
        callback_count++;
    });
    EXPECT_GT(id1, 0);

    auto id2 = driver.Subscribe([&](const SimulationState&) {
        callback_count++;
    });
    EXPECT_GT(id2, id1);

    // Unsubscribe
    EXPECT_TRUE(driver.Unsubscribe(id1));
    EXPECT_FALSE(driver.Unsubscribe(id1));  // Already removed

    EXPECT_TRUE(driver.Unsubscribe(id2));
}

// Test raw subscriber management
TEST(DriverTest, RawSubscriberManagement) {
    Driver driver;

    std::atomic<int> callback_count{0};

    // Subscribe with raw callback
    auto id1 = driver.SubscribeRaw([&](std::span<const uint8_t>) {
        callback_count++;
    });
    EXPECT_GT(id1, 0);

    auto id2 = driver.SubscribeRaw([&](std::span<const uint8_t>) {
        callback_count++;
    });
    EXPECT_GT(id2, id1);

    // Mix of raw and parsed subscribers
    auto id3 = driver.Subscribe([&](const SimulationState&) {
        callback_count++;
    });
    EXPECT_GT(id3, id2);

    // Unsubscribe
    EXPECT_TRUE(driver.Unsubscribe(id1));
    EXPECT_FALSE(driver.Unsubscribe(id1));  // Already removed

    EXPECT_TRUE(driver.Unsubscribe(id2));
    EXPECT_TRUE(driver.Unsubscribe(id3));
}

// Test receive control
TEST(DriverTest, ReceiveControl) {
    DriverConfig config;
    config.auto_start_receiving = false;

    Driver driver(config);

    EXPECT_FALSE(driver.IsReceiving());

    driver.StartReceiving();
    // Note: receiving won't actually start until connected
    // but the flag should be set
    EXPECT_TRUE(driver.IsReceiving());

    driver.StopReceiving();
    EXPECT_FALSE(driver.IsReceiving());
}

// Test error callback
TEST(DriverTest, ErrorCallback) {
    Driver driver;

    std::atomic<bool> error_received{false};
    driver.OnError([&](std::error_code, const std::string&) {
        error_received = true;
    });

    // Error callback is set but won't be called without actual errors
    EXPECT_FALSE(error_received);
}

// ============================================================================
// Force Data / StatePacket Tests
// ============================================================================

// Default SimulationState has empty force fields
TEST(DriverTest, ForceDataDefaults) {
    SimulationState state;

    EXPECT_TRUE(state.qfrc_actuator.empty());
    EXPECT_TRUE(state.qfrc_bias.empty());
    EXPECT_TRUE(state.cfrc_int.empty());
    EXPECT_TRUE(state.cfrc_ext.empty());
    EXPECT_EQ(state.ncon, 0);
    EXPECT_TRUE(state.contact_pos.empty());
    EXPECT_TRUE(state.contact_force.empty());
    EXPECT_TRUE(state.contact_body1.empty());
    EXPECT_TRUE(state.contact_body2.empty());
}

// Round-trip: serialize with force data, deserialize, verify all fields
TEST(DriverTest, ForceDataRoundTrip) {
    // Build a state with force data
    SimulationState original;
    original.sequence = 42;
    original.time = 1.5;
    original.qpos = {1.0, 2.0, 3.0};
    original.qvel = {0.1, 0.2, 0.3};
    original.ctrl = {10.0, 20.0};
    original.step_index = 100;

    // Force data (nv=3, nbody=2)
    original.qfrc_actuator = {1.1, 2.2, 3.3};
    original.qfrc_bias = {-9.8, 0.0, 0.0};
    original.cfrc_int = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};  // 2 bodies * 6
    original.cfrc_ext = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};

    // Contact data (ncon=2)
    original.ncon = 2;
    original.contact_pos = {0.0, 0.0, 0.0, 1.0, 1.0, 0.0};  // 2 contacts * 3
    original.contact_force = {50.0, 30.0};
    original.contact_body1 = {0, 0};
    original.contact_body2 = {1, 1};

    // Serialize via FlatBuffers Pack
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = imujoco::schema::StatePacket::Pack(builder, &original);
    builder.Finish(offset, imujoco::schema::StatePacketIdentifier());

    // Verify buffer
    auto buf = builder.GetBufferPointer();
    auto size = builder.GetSize();
    {
        flatbuffers::Verifier verifier(buf, size);
        ASSERT_TRUE(imujoco::schema::VerifyStatePacketBuffer(verifier));
    }

    // Deserialize
    SimulationState deserialized;
    auto packet = imujoco::schema::GetStatePacket(buf);
    packet->UnPackTo(&deserialized);

    // Verify old fields
    EXPECT_EQ(deserialized.sequence, 42);
    EXPECT_DOUBLE_EQ(deserialized.time, 1.5);
    EXPECT_EQ(deserialized.qpos.size(), 3);
    EXPECT_EQ(deserialized.step_index, 100);

    // Verify force fields
    ASSERT_EQ(deserialized.qfrc_actuator.size(), 3);
    EXPECT_DOUBLE_EQ(deserialized.qfrc_actuator[0], 1.1);
    EXPECT_DOUBLE_EQ(deserialized.qfrc_actuator[2], 3.3);

    ASSERT_EQ(deserialized.qfrc_bias.size(), 3);
    EXPECT_DOUBLE_EQ(deserialized.qfrc_bias[0], -9.8);

    ASSERT_EQ(deserialized.cfrc_int.size(), 12);
    EXPECT_DOUBLE_EQ(deserialized.cfrc_int[5], 6.0);

    ASSERT_EQ(deserialized.cfrc_ext.size(), 12);
    EXPECT_DOUBLE_EQ(deserialized.cfrc_ext[6], 1.0);

    // Verify contact fields
    EXPECT_EQ(deserialized.ncon, 2);
    ASSERT_EQ(deserialized.contact_pos.size(), 6);
    EXPECT_DOUBLE_EQ(deserialized.contact_pos[3], 1.0);
    ASSERT_EQ(deserialized.contact_force.size(), 2);
    EXPECT_DOUBLE_EQ(deserialized.contact_force[0], 50.0);
    ASSERT_EQ(deserialized.contact_body1.size(), 2);
    EXPECT_EQ(deserialized.contact_body1[0], 0);
    ASSERT_EQ(deserialized.contact_body2.size(), 2);
    EXPECT_EQ(deserialized.contact_body2[1], 1);
}

// Forward compat: new driver reads packet WITHOUT force data (sendForceData=false)
TEST(DriverTest, ForceDataForwardCompat) {
    // Build state with only old fields
    SimulationState original;
    original.sequence = 7;
    original.time = 0.5;
    original.qpos = {1.0, 2.0};
    // Leave all force fields at defaults

    flatbuffers::FlatBufferBuilder builder(512);
    auto offset = imujoco::schema::StatePacket::Pack(builder, &original);
    builder.Finish(offset, imujoco::schema::StatePacketIdentifier());

    auto buf = builder.GetBufferPointer();
    auto size = builder.GetSize();
    {
        flatbuffers::Verifier verifier(buf, size);
        ASSERT_TRUE(imujoco::schema::VerifyStatePacketBuffer(verifier));
    }

    SimulationState deserialized;
    imujoco::schema::GetStatePacket(buf)->UnPackTo(&deserialized);

    // Old fields work
    EXPECT_EQ(deserialized.sequence, 7);
    EXPECT_EQ(deserialized.qpos.size(), 2);

    // New fields default to empty/zero
    EXPECT_TRUE(deserialized.qfrc_actuator.empty());
    EXPECT_TRUE(deserialized.qfrc_bias.empty());
    EXPECT_TRUE(deserialized.cfrc_int.empty());
    EXPECT_TRUE(deserialized.cfrc_ext.empty());
    EXPECT_EQ(deserialized.ncon, 0);
    EXPECT_TRUE(deserialized.contact_pos.empty());
    EXPECT_TRUE(deserialized.contact_force.empty());
    EXPECT_TRUE(deserialized.contact_body1.empty());
    EXPECT_TRUE(deserialized.contact_body2.empty());
}

// ============================================================================
// Integration Tests
// ============================================================================

// Integration test placeholder - requires running iMuJoCo
// This test is disabled by default
TEST(DriverTest, DISABLED_ConnectToSimulation) {
    DriverConfig config;
    config.host = "127.0.0.1";
    config.port = 9000;
    config.timeout_ms = 1000;

    Driver driver(config);

    ASSERT_TRUE(driver.Connect());
    EXPECT_TRUE(driver.IsConnected());

    // Subscribe to state updates
    std::atomic<int> state_count{0};
    driver.Subscribe([&](const SimulationState& state) {
        state_count++;
        EXPECT_GE(state.time, 0.0);
    });

    // Send control commands (async)
    std::vector<double> ctrl(21, 0.0);  // Humanoid has 21 actuators
    for (int i = 0; i < 100; i++) {
        driver.SendControl(ctrl);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait for callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_GT(state_count.load(), 0);

    driver.Disconnect();
    EXPECT_FALSE(driver.IsConnected());
}

}  // namespace
}  // namespace imujoco::driver
