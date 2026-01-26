// driver_test.cc
// Unit tests for MuJoCo Driver

#include "imujoco/driver/driver.h"

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
    EXPECT_EQ(config.port, 8888);
    EXPECT_EQ(config.local_port, 0);
    EXPECT_EQ(config.timeout_ms, 100);
    EXPECT_DOUBLE_EQ(config.max_control_rate, 1000.0);
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

// Integration test placeholder - requires running iMuJoCo
// This test is disabled by default
TEST(DriverTest, DISABLED_ConnectToSimulation) {
    DriverConfig config;
    config.host = "127.0.0.1";
    config.port = 8888;
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
