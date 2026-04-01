// grpc_tester.cc
// Simple gRPC client for testing the SimulationControl service.
//
// Usage:
//   grpc_tester --target localhost:8999
//   grpc_tester --target localhost:8999 --command list
//   grpc_tester --target localhost:8999 --command load --instance 1 --model scene
//   grpc_tester --target localhost:8999 --smoke --instance 1 --model scene
//
// Commands:
//   list          List all simulation instances
//   info          Get info for --instance
//   load          Load --model on --instance
//   unload        Unload --instance
//   start         Start --instance
//   pause         Pause --instance
//   reset         Reset --instance
//   reset_kf      Reset --instance to --keyframe
//   step          Step --instance one tick
//   state         Get physics state for --instance
//   set_mode      Set control mode on --instance (--model = live/paced_replay/sim_time_guarded/ptp_scheduled)
//   smoke         Run full lifecycle smoke test (default)
//   race          Race test: ListInstances vs Load/Unload (crash regression)

#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdlib>

#include <grpcpp/grpcpp.h>
#include "schema/simulation_control.grpc.pb.h"
#include "schema/simulation_control.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using namespace imujoco;

// Minimal --key value argument parser (header-only, no deps).
class Args {
public:
    Args(int argc, char* argv[]) {
        program_ = (argc > 0) ? argv[0] : "grpc_tester";
        for (int i = 1; i < argc; i++) {
            std::string a = argv[i];
            if (a.size() > 2 && a[0] == '-' && a[1] == '-') {
                std::string k = a.substr(2);
                if (i + 1 < argc && !(argv[i+1][0] == '-' && argv[i+1][1] == '-'))
                    args_[k] = argv[++i];
                else
                    args_[k] = "";
            }
        }
    }
    std::string get(const std::string& k, const std::string& d) const {
        auto it = args_.find(k); return it != args_.end() ? it->second : d;
    }
    int get_int(const std::string& k, int d) const {
        auto it = args_.find(k);
        return (it != args_.end() && !it->second.empty()) ? std::atoi(it->second.c_str()) : d;
    }
    bool has(const std::string& k) const { return args_.count(k); }
    const std::string& program() const { return program_; }
private:
    std::map<std::string, std::string> args_;
    std::string program_;
};

// ============================================================================
// Helpers
// ============================================================================

static const char* StateName(SimulationState s) {
    switch (s) {
        case SIMULATION_STATE_UNSPECIFIED: return "UNSPECIFIED";
        case SIMULATION_STATE_INACTIVE:   return "INACTIVE";
        case SIMULATION_STATE_LOADED:     return "LOADED";
        case SIMULATION_STATE_RUNNING:    return "RUNNING";
        case SIMULATION_STATE_PAUSED:     return "PAUSED";
        default:                          return "UNKNOWN";
    }
}

static void PrintStatus(const std::string& rpc, const Status& status) {
    if (status.ok()) {
        std::cout << "  [OK] " << rpc << "\n";
    } else {
        std::cerr << "  [FAIL] " << rpc << ": "
                  << status.error_code() << " — " << status.error_message()
                  << "\n";
    }
}

static void PrintControlResponse(const std::string& rpc,
                                 const Status& status,
                                 const ControlResponse& resp) {
    PrintStatus(rpc, status);
    if (status.ok()) {
        std::cout << "    success: " << (resp.success() ? "true" : "false")
                  << "\n";
        if (!resp.error().empty()) {
            std::cout << "    error:   " << resp.error() << "\n";
        }
    }
}

static void PrintInstanceInfo(const InstanceInfo& info) {
    std::cout << "  Instance " << info.instance_id()
              << "  state=" << StateName(info.state())
              << "  model=\"" << info.model_name() << "\""
              << "  udp_port=" << info.udp_port()
              << "\n";
    if (info.has_stats()) {
        const auto& s = info.stats();
        std::cout << "    SPS=" << s.steps_per_second()
                  << "  tx=" << std::fixed << std::setprecision(1)
                  << s.tx_rate() << " pkt/s"
                  << "  rx=" << s.rx_rate() << " pkt/s"
                  << "  sent=" << s.packets_sent()
                  << "  recv=" << s.packets_received()
                  << "  client=" << (s.has_client() ? "yes" : "no")
                  << "\n";
    }
}

static void PrintPhysicsState(const PhysicsState& ps) {
    std::cout << "    time=" << std::fixed << std::setprecision(4)
              << ps.time() << "\n";
    std::cout << "    energy: PE=" << ps.energy_potential()
              << "  KE=" << ps.energy_kinetic() << "\n";

    auto printVec = [](const char* label,
                       const google::protobuf::RepeatedField<double>& v,
                       int max_show = 8) {
        std::cout << "    " << label << "[" << v.size() << "]: [";
        int n = std::min(max_show, v.size());
        for (int i = 0; i < n; i++) {
            if (i > 0) std::cout << ", ";
            std::cout << std::fixed << std::setprecision(4) << v[i];
        }
        if (v.size() > max_show) std::cout << ", ...";
        std::cout << "]\n";
    };

    printVec("qpos",       ps.qpos());
    printVec("qvel",       ps.qvel());
    printVec("ctrl",       ps.ctrl());
    printVec("sensordata", ps.sensordata());
}

// ============================================================================
// RPC wrappers
// ============================================================================

class Tester {
public:
    explicit Tester(std::shared_ptr<Channel> channel, int timeout_ms,
                    int load_timeout_ms)
        : channel_(channel),
          stub_(SimulationControl::NewStub(channel)),
          timeout_ms_(timeout_ms),
          load_timeout_ms_(load_timeout_ms) {}

    // --- Instance discovery ---

    bool ListInstances() {
        ListInstancesRequest req;
        ListInstancesResponse resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->ListInstances(&ctx, req, &resp);
        PrintStatus("ListInstances", status);
        if (status.ok()) {
            std::cout << "  " << resp.instances_size() << " instance(s):\n";
            for (const auto& info : resp.instances()) {
                PrintInstanceInfo(info);
            }
        }
        return status.ok();
    }

    bool GetInstanceInfo(int id) {
        GetInstanceInfoRequest req;
        req.set_instance_id(id);
        InstanceInfo resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->GetInstanceInfo(&ctx, req, &resp);
        PrintStatus("GetInstanceInfo", status);
        if (status.ok()) {
            PrintInstanceInfo(resp);
        }
        return status.ok();
    }

    // --- Lifecycle ---
    // LoadModel and Unload block server-side until the operation completes
    // (Swift MainActor work + model parsing/mesh extraction). A client-side
    // deadline is applied via --load_timeout; the server response is the
    // completion signal for these operations.

    bool LoadModel(int id, const std::string& model) {
        LoadModelRequest req;
        req.set_instance_id(id);
        req.set_model_name(model);
        ControlResponse resp;
        ClientContext ctx;
        SetLoadDeadline(ctx);

        Status status = stub_->LoadModel(&ctx, req, &resp);
        PrintControlResponse("LoadModel", status, resp);
        return status.ok() && resp.success();
    }

    bool Unload(int id) {
        InstanceRequest req;
        req.set_instance_id(id);
        ControlResponse resp;
        ClientContext ctx;
        SetLoadDeadline(ctx);

        Status status = stub_->Unload(&ctx, req, &resp);
        PrintControlResponse("Unload", status, resp);
        return status.ok() && resp.success();
    }

    bool Start(int id) {
        InstanceRequest req;
        req.set_instance_id(id);
        ControlResponse resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->Start(&ctx, req, &resp);
        PrintControlResponse("Start", status, resp);
        return status.ok() && resp.success();
    }

    bool Pause(int id) {
        InstanceRequest req;
        req.set_instance_id(id);
        ControlResponse resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->Pause(&ctx, req, &resp);
        PrintControlResponse("Pause", status, resp);
        return status.ok() && resp.success();
    }

    bool Reset(int id) {
        InstanceRequest req;
        req.set_instance_id(id);
        ControlResponse resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->Reset(&ctx, req, &resp);
        PrintControlResponse("Reset", status, resp);
        return status.ok() && resp.success();
    }

    bool ResetToKeyframe(int id, int keyframe) {
        ResetToKeyframeRequest req;
        req.set_instance_id(id);
        req.set_keyframe_index(keyframe);
        ControlResponse resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->ResetToKeyframe(&ctx, req, &resp);
        PrintControlResponse("ResetToKeyframe", status, resp);
        return status.ok() && resp.success();
    }

    bool Step(int id) {
        InstanceRequest req;
        req.set_instance_id(id);
        ControlResponse resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->Step(&ctx, req, &resp);
        PrintControlResponse("Step", status, resp);
        return status.ok() && resp.success();
    }

    // --- Control mode ---

    bool SetControlMode(int id, const std::string& mode_str) {
        SetControlModeRequest req;
        req.set_instance_id(id);

        ControlMode mode = CONTROL_MODE_LIVE;
        if (mode_str == "live" || mode_str == "0") mode = CONTROL_MODE_LIVE;
        else if (mode_str == "paced_replay" || mode_str == "1") mode = CONTROL_MODE_PACED_REPLAY;
        else if (mode_str == "sim_time_guarded" || mode_str == "2") mode = CONTROL_MODE_SIM_TIME_GUARDED;
        else if (mode_str == "ptp_scheduled" || mode_str == "3") mode = CONTROL_MODE_PTP_SCHEDULED;
        else {
            std::cerr << "  Unknown mode: " << mode_str
                      << " (use: live, paced_replay, sim_time_guarded, ptp_scheduled)\n";
            return false;
        }

        req.set_mode(mode);
        ControlResponse resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->SetControlMode(&ctx, req, &resp);
        PrintControlResponse("SetControlMode(" + mode_str + ")", status, resp);
        return status.ok() && resp.success();
    }

    // --- State query ---

    bool GetState(int id) {
        InstanceRequest req;
        req.set_instance_id(id);
        PhysicsState resp;
        ClientContext ctx;
        SetDeadline(ctx);

        Status status = stub_->GetState(&ctx, req, &resp);
        PrintStatus("GetState", status);
        if (status.ok()) {
            PrintPhysicsState(resp);
        }
        return status.ok();
    }

    // --- Smoke test ---

    bool RunSmokeTest(int id, const std::string& model) {
        std::cout << "\n=== Smoke test: instance " << id
                  << ", model \"" << model << "\" ===\n\n";
        int pass = 0, fail = 0;
        auto check = [&](bool ok) { ok ? pass++ : fail++; };

        std::cout << "1. ListInstances\n";
        check(ListInstances());

        std::cout << "\n2. LoadModel (blocking until done)\n";
        check(LoadModel(id, model));

        std::cout << "\n3. GetInstanceInfo\n";
        check(GetInstanceInfo(id));

        std::cout << "\n4. Start\n";
        check(Start(id));

        // Let it run briefly so we get non-zero state.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "\n5. GetState (running)\n";
        check(GetState(id));

        std::cout << "\n6. Pause\n";
        check(Pause(id));

        std::cout << "\n7. Step (single)\n";
        check(Step(id));

        std::cout << "\n8. GetState (after step)\n";
        check(GetState(id));

        std::cout << "\n9. Reset\n";
        check(Reset(id));

        std::cout << "\n10. GetState (after reset)\n";
        check(GetState(id));

        std::cout << "\n11. Unload (blocking until done)\n";
        check(Unload(id));

        std::cout << "\n=== Results: " << pass << " passed, "
                  << fail << " failed ===\n";
        return fail == 0;
    }

    // --- Race test: ListInstances/GetInstanceInfo vs Load/Unload ---
    // Reproduces the crash fixed in feat/grpc-mainactor-serialization where
    // ListInstances accessed a freed C++ runtime pointer on the gRPC thread
    // while Unload freed it on MainActor. After the fix, all RPCs are routed
    // through MainActor callbacks, so this test should pass without crashes.

    bool RunRaceTest(int id, const std::string& model, int cycles) {
        std::cout << "\n=== Race test: instance " << id
                  << ", model \"" << model << "\""
                  << ", " << cycles << " load/unload cycles ===\n\n";

        std::atomic<bool> stop{false};
        std::atomic<int> listOk{0}, listFail{0};
        std::atomic<int> infoOk{0}, infoFail{0};

        // Background thread: hammer ListInstances continuously
        std::thread listThread([&]() {
            // Each thread needs its own stub (ClientContext is not thread-safe)
            auto threadStub = SimulationControl::NewStub(channel_);
            while (!stop.load(std::memory_order_relaxed)) {
                ListInstancesRequest req;
                ListInstancesResponse resp;
                ClientContext ctx;
                ctx.set_deadline(std::chrono::system_clock::now() +
                                 std::chrono::milliseconds(timeout_ms_));
                Status status = threadStub->ListInstances(&ctx, req, &resp);
                if (status.ok()) {
                    listOk.fetch_add(1, std::memory_order_relaxed);
                } else {
                    listFail.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });

        // Background thread: hammer GetInstanceInfo continuously
        std::thread infoThread([&]() {
            auto threadStub = SimulationControl::NewStub(channel_);
            while (!stop.load(std::memory_order_relaxed)) {
                GetInstanceInfoRequest req;
                req.set_instance_id(id);
                InstanceInfo resp;
                ClientContext ctx;
                ctx.set_deadline(std::chrono::system_clock::now() +
                                 std::chrono::milliseconds(timeout_ms_));
                Status status = threadStub->GetInstanceInfo(&ctx, req, &resp);
                if (status.ok()) {
                    infoOk.fetch_add(1, std::memory_order_relaxed);
                } else {
                    infoFail.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });

        // Main thread: Load/Unload cycles
        int loadFail = 0;
        for (int i = 0; i < cycles; i++) {
            std::cout << "  Cycle " << (i + 1) << "/" << cycles << ": ";

            std::cout << "load... ";
            if (!LoadModel(id, model)) {
                std::cout << "LOAD FAILED\n";
                loadFail++;
                continue;
            }

            // Let background threads race against a loaded instance
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "unload... ";
            if (!Unload(id)) {
                std::cout << "UNLOAD FAILED\n";
                loadFail++;
                continue;
            }

            std::cout << "done\n";
        }

        // Stop background threads
        stop.store(true, std::memory_order_relaxed);
        listThread.join();
        infoThread.join();

        int totalList = listOk.load() + listFail.load();
        int totalInfo = infoOk.load() + infoFail.load();

        std::cout << "\n=== Race test results ===\n"
                  << "  Load/Unload cycles: " << cycles
                  << " (" << loadFail << " failed)\n"
                  << "  ListInstances:      " << totalList << " calls ("
                  << listOk.load() << " ok, " << listFail.load() << " fail)\n"
                  << "  GetInstanceInfo:    " << totalInfo << " calls ("
                  << infoOk.load() << " ok, " << infoFail.load() << " fail)\n";

        bool ok = (loadFail == 0 && listFail.load() == 0 && infoFail.load() == 0);
        std::cout << "  Overall: " << (ok ? "PASS" : "FAIL")
                  << " (no crashes = fix verified)\n";
        return ok;
    }

private:
    void SetDeadline(ClientContext& ctx) {
        ctx.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(timeout_ms_));
    }

    void SetLoadDeadline(ClientContext& ctx) {
        ctx.set_deadline(std::chrono::system_clock::now() +
                         std::chrono::milliseconds(load_timeout_ms_));
    }

    std::shared_ptr<Channel> channel_;
    std::unique_ptr<SimulationControl::Stub> stub_;
    int timeout_ms_;
    int load_timeout_ms_;
};

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    Args args(argc, argv);

    if (args.has("help")) {
        std::cout << "Usage: " << args.program()
                  << " [--target HOST:PORT] [--command CMD] [options]\n\n"
                  << "Options:\n"
                  << "  --target    gRPC server address (default: localhost:8999)\n"
                  << "  --command   RPC to call: list, info, load, unload, start,\n"
                  << "              pause, reset, reset_kf, step, state, smoke\n"
                  << "  --instance  Instance ID (default: 1)\n"
                  << "  --model     Model name for load (default: scene)\n"
                  << "  --keyframe  Keyframe index for reset_kf (default: 0)\n"
                  << "  --timeout      RPC deadline in ms (default: 5000)\n"
                  << "  --load_timeout Deadline for LoadModel/Unload in ms (default: 120000)\n"
                  << "  --smoke     Shorthand for --command smoke\n"
                  << "  --cycles    Number of load/unload cycles for race test (default: 5)\n";
        return 0;
    }

    std::string target = args.get("target", "localhost:8999");
    std::string command = args.get("command", "");
    int instance = args.get_int("instance", 1);
    std::string model = args.get("model", "scene");
    int keyframe = args.get_int("keyframe", 0);
    int timeout_ms = args.get_int("timeout", 5000);

    int load_timeout_ms = args.get_int("load_timeout", 120000);
    int cycles = args.get_int("cycles", 5);

    if (args.has("smoke")) {
        command = "smoke";
    }
    if (command.empty()) {
        command = "smoke";
    }

    std::cout << "gRPC SimulationControl Tester\n"
              << "Target: " << target << "\n\n";

    auto channel = grpc::CreateChannel(target,
                                       grpc::InsecureChannelCredentials());
    Tester tester(channel, timeout_ms, load_timeout_ms);

    bool ok = true;
    if (command == "list") {
        ok = tester.ListInstances();
    } else if (command == "info") {
        ok = tester.GetInstanceInfo(instance);
    } else if (command == "load") {
        ok = tester.LoadModel(instance, model);
    } else if (command == "unload") {
        ok = tester.Unload(instance);
    } else if (command == "start") {
        ok = tester.Start(instance);
    } else if (command == "pause") {
        ok = tester.Pause(instance);
    } else if (command == "reset") {
        ok = tester.Reset(instance);
    } else if (command == "reset_kf") {
        ok = tester.ResetToKeyframe(instance, keyframe);
    } else if (command == "step") {
        ok = tester.Step(instance);
    } else if (command == "state") {
        ok = tester.GetState(instance);
    } else if (command == "set_mode") {
        ok = tester.SetControlMode(instance, model);
    } else if (command == "smoke") {
        ok = tester.RunSmokeTest(instance, model);
    } else if (command == "race") {
        ok = tester.RunRaceTest(instance, model, cycles);
    } else {
        std::cerr << "Unknown command: " << command << "\n";
        return 1;
    }

    return ok ? 0 : 1;
}
