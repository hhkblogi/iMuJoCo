// simple_control.cc
// Example: Async control loop with iMuJoCo simulation
//
// This example demonstrates the async model:
// - RX thread receives state updates via subscription
// - TX sends control commands from main thread
// - Both run in parallel (full-duplex UDP)

#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <mutex>

#include "imujoco/driver/driver.h"

using namespace imujoco::driver;

// Global flag for signal handling
std::atomic<bool> running{true};

void signalHandler(int) {
    running = false;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string host = "127.0.0.1";
    uint16_t port = 8888;

    if (argc > 1) {
        host = argv[1];
    }
    if (argc > 2) {
        try {
            int parsed_port = std::stoi(argv[2]);
            if (parsed_port < 0 || parsed_port > 65535) {
                std::cerr << "Invalid port number: must be in range [0, 65535]" << std::endl;
                return 1;
            }
            port = static_cast<uint16_t>(parsed_port);
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse port '" << argv[2] << "': " << e.what() << std::endl;
            return 1;
        }
    }

    std::cout << "MuJoCo Driver Example (Async Mode)" << std::endl;
    std::cout << "Connecting to " << host << ":" << port << "..." << std::endl;

    // Configure driver
    DriverConfig config;
    config.host = host;
    config.port = port;
    config.timeout_ms = 100;

    // Create driver (RX thread starts automatically on connect)
    Driver driver(config);

    // Set up error callback
    driver.OnError([](std::error_code ec, const std::string& msg) {
        std::cerr << "Error: " << msg;
        if (ec) {
            std::cerr << " (" << ec.message() << ")";
        }
        std::cerr << "\n";
    });

    // Shared state between RX callback and main thread
    std::mutex state_mutex;
    double latest_time = 0.0;
    std::atomic<size_t> ctrl_dim{0};
    std::atomic<int> state_count{0};

    // Subscribe to state updates (runs on RX thread)
    driver.Subscribe([&](const SimulationState& state) {
        state_count++;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            latest_time = state.time;
        }

        // Set ctrl_dim once (atomic)
        if (ctrl_dim.load() == 0 && !state.ctrl.empty()) {
            ctrl_dim.store(state.ctrl.size());
        }

        // Print progress every 100 states
        if (state_count % 100 == 0) {
            std::cout << "[RX] State #" << state_count
                      << " | Time: " << state.time
                      << " | KE: " << state.energy_kinetic << "\n";
        }
    });

    // Connect to simulation
    if (!driver.Connect()) {
        std::cerr << "Failed to connect to simulation\n";
        return 1;
    }

    std::cout << "Connected!" << std::endl;

    // Set up signal handler for clean exit
    std::signal(SIGINT, signalHandler);

    // Wait for first state to determine control dimension
    // Send initial control packets to trigger state responses from server
    std::cout << "Waiting for first state..." << std::endl;
    std::vector<double> init_ctrl(32, 0.0);  // Send zeros initially
    int attempts = 0;
    while (running && ctrl_dim.load() == 0 && attempts < 100) {
        driver.SendControl(init_ctrl);
        attempts++;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    size_t local_ctrl_dim = ctrl_dim.load();
    if (local_ctrl_dim == 0) {
        std::cerr << "Timeout waiting for first state\n";
        driver.Disconnect();
        return 1;
    }

    std::cout << "Control dimension: " << local_ctrl_dim << std::endl;

    // Control loop - sends at ~1000 Hz
    int send_count = 0;
    std::vector<double> ctrl(local_ctrl_dim, 0.0);

    while (running) {
        // Get latest time for control computation
        double t;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            t = latest_time;
        }

        // Generate control signal (example: sinusoidal)
        for (size_t i = 0; i < ctrl.size(); i++) {
            ctrl[i] = 0.1 * std::sin(2.0 * M_PI * 0.5 * t + i * 0.1);
        }

        // Send control (non-blocking)
        driver.SendControl(ctrl);
        send_count++;

        // Print progress every 1000 sends
        if (send_count % 1000 == 0) {
            auto stats = driver.GetStats();
            std::cout << "[TX] Sent: " << send_count
                      << " | TX pkts: " << stats.packets_sent
                      << " | RX pkts: " << stats.packets_received
                      << "\n";
        }

        // Control rate (~1000 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\nExiting.\n";
    std::cout << "  Commands sent: " << send_count << "\n";
    std::cout << "  States received: " << state_count.load() << "\n";

    auto final_stats = driver.GetStats();
    std::cout << "Final stats:\n";
    std::cout << "  Packets sent: " << final_stats.packets_sent << "\n";
    std::cout << "  Packets received: " << final_stats.packets_received << "\n";
    std::cout << "  Send errors: " << final_stats.send_errors << "\n";
    std::cout << "  Receive errors: " << final_stats.receive_errors << "\n";

    driver.Disconnect();

    return 0;
}
