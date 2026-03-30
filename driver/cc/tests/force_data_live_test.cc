// Minimal live test for force data — connects to running iMuJoCo,
// receives state packets, and verifies force fields.
// Usage: bazel run //driver:force_data_live_test -- 192.168.65.102 9001

#include "imujoco/driver/driver.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

using namespace imujoco::driver;

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <host> <port>\n", argv[0]);
        return 1;
    }

    DriverConfig config;
    config.host = argv[1];
    config.port = static_cast<uint16_t>(std::atoi(argv[2]));

    Driver driver(config);
    if (!driver.Connect()) {
        fprintf(stderr, "Failed to connect to %s:%d\n", argv[1], std::atoi(argv[2]));
        return 1;
    }

    std::atomic<int> count{0};
    SimulationState last_state;

    auto sub_id = driver.Subscribe([&](const SimulationState& state) {
        last_state = state;  // Copy the full state
        count.fetch_add(1);
    });

    printf("Connected. Sending control packets...\n");

    for (int i = 0; i < 50 && count.load() < 10; i++) {
        driver.SendControl(std::span<const double>{});
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    driver.Unsubscribe(sub_id);

    int n = count.load();
    printf("Received %d state packets\n\n", n);

    if (n == 0) {
        printf("FAIL: No packets received\n");
        driver.Disconnect();
        return 1;
    }

    // Verify force fields from last state
    auto& s = last_state;
    printf("seq=%u time=%.4f\n", s.sequence, s.time);
    printf("qpos[%zu] qvel[%zu] ctrl[%zu]\n",
           s.qpos.size(), s.qvel.size(), s.ctrl.size());
    printf("\n=== Force Data ===\n");
    printf("qfrc_actuator[%zu]\n", s.qfrc_actuator.size());
    printf("qfrc_bias[%zu]\n", s.qfrc_bias.size());
    printf("cfrc_int[%zu]\n", s.cfrc_int.size());
    printf("cfrc_ext[%zu]\n", s.cfrc_ext.size());
    printf("ncon=%d\n", s.ncon);
    printf("contact_pos[%zu]\n", s.contact_pos.size());
    printf("contact_force[%zu]\n", s.contact_force.size());
    printf("contact_body1[%zu]\n", s.contact_body1.size());
    printf("contact_body2[%zu]\n", s.contact_body2.size());

    int errors = 0;
    if (s.qfrc_bias.empty()) { printf("FAIL: qfrc_bias empty\n"); errors++; }
    if (s.cfrc_int.empty()) { printf("FAIL: cfrc_int empty\n"); errors++; }
    if (s.ncon > 0) {
        if (s.contact_pos.size() != static_cast<size_t>(s.ncon * 3)) {
            printf("FAIL: contact_pos size %zu != ncon*3=%d\n", s.contact_pos.size(), s.ncon*3);
            errors++;
        }
        if (s.contact_force.size() != static_cast<size_t>(s.ncon)) {
            printf("FAIL: contact_force size %zu != ncon=%d\n", s.contact_force.size(), s.ncon);
            errors++;
        }
        if (s.contact_body1.size() != static_cast<size_t>(s.ncon)) {
            printf("FAIL: contact_body1 size %zu != ncon=%d\n", s.contact_body1.size(), s.ncon);
            errors++;
        }
        // Print first few contact forces
        printf("\ncontact_force: [");
        for (size_t i = 0; i < s.contact_force.size() && i < 5; i++) {
            printf("%.2f%s", s.contact_force[i], i < s.contact_force.size()-1 ? ", " : "");
        }
        printf("]\n");
    }

    printf("\n%s\n", errors == 0 ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    driver.Disconnect();
    return errors;
}
