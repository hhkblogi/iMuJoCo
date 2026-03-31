// bindings.cc
// Python bindings for imujoco_driver using pybind11

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <memory>

#include "imujoco/driver/driver.h"

namespace py = pybind11;
using namespace imujoco::driver;

PYBIND11_MODULE(_imujoco_driver, m) {
    m.doc() = R"doc(iMuJoCo Driver - Thread-safe UDP client for iMuJoCo simulation.

Threading Model:
  - TX (send_control): Runs on caller's thread, GIL released during send
  - RX (callbacks): Runs on dedicated RX thread, GIL acquired for Python callbacks
  - All public methods are thread-safe

Important: State callbacks run on the RX thread. Keep callbacks lightweight
or dispatch work to your own thread to avoid blocking state reception.
)doc";

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif

    // ClockSyncState
    py::class_<ClockSyncState>(m, "ClockSyncState")
        .def(py::init<>())
        .def_readonly("locked", &ClockSyncState::locked,
            "True when clock servo has converged")
        .def_readonly("offset_us", &ClockSyncState::offset_us,
            "Clock offset in microseconds (device = driver + offset)")
        .def_readonly("delay_us", &ClockSyncState::delay_us,
            "One-way delay estimate in microseconds")
        .def_readonly("rate_ratio_ppm", &ClockSyncState::rate_ratio_ppm,
            "Clock rate ratio in PPM")
        .def_readonly("exchanges", &ClockSyncState::exchanges,
            "Total peer-delay exchanges processed")
        .def_readonly("jitter_us", &ClockSyncState::jitter_us,
            "Windowed PI output jitter in microseconds (used for lock detection)")
        .def_readonly("raw_jitter_us", &ClockSyncState::raw_jitter_us,
            "Full-history Welford jitter of raw offsets in microseconds (diagnostic)")
        .def_readonly("outliers_rejected", &ClockSyncState::outliers_rejected,
            "Total outlier samples rejected")
        .def_readonly("pi_integral", &ClockSyncState::pi_integral,
            "PI controller integral term")
        .def_readonly("mean_offset_us", &ClockSyncState::mean_offset_us,
            "Welford running mean offset in microseconds")
        .def_readonly("lock_count", &ClockSyncState::lock_count,
            "Number of times lock was achieved")
        .def_readonly("unlock_count", &ClockSyncState::unlock_count,
            "Number of times lock was lost")
        .def("__repr__", [](const ClockSyncState& s) {
            return "<ClockSyncState locked=" + std::string(s.locked ? "True" : "False") +
                   " offset=" + std::to_string(s.offset_us) + "us" +
                   " delay=" + std::to_string(s.delay_us) + "us" +
                   " exchanges=" + std::to_string(s.exchanges) + ">";
        });

    // DriverConfig
    py::class_<DriverConfig>(m, "DriverConfig")
        .def(py::init<>())
        .def_readwrite("host", &DriverConfig::host,
            "Target simulation host (default: 127.0.0.1)")
        .def_readwrite("port", &DriverConfig::port,
            "Target simulation UDP port (default: 9000)")
        .def_readwrite("local_port", &DriverConfig::local_port,
            "Local bind port (0 = ephemeral)")
        .def_readwrite("timeout_ms", &DriverConfig::timeout_ms,
            "Receive timeout in milliseconds (0 = no timeout)")
        .def_readwrite("auto_start_receiving", &DriverConfig::auto_start_receiving,
            "Auto-start receiving on connect (default: true)")
        .def_readwrite("enable_sync", &DriverConfig::enable_sync,
            "Enable PTP time sync (default: false, set true for PTPScheduled mode)")
        .def_readwrite("sync_port", &DriverConfig::sync_port,
            "Sync port (0 = control port + 1000)")
        .def_readwrite("sync_interval_ms", &DriverConfig::sync_interval_ms,
            "Sync exchange interval in ms (default: 100)")
        .def("__repr__", [](const DriverConfig& c) {
            return "<DriverConfig host='" + c.host + "' port=" + std::to_string(c.port) + ">";
        });

    // DriverStats
    py::class_<DriverStats>(m, "DriverStats")
        .def(py::init<>())
        .def_readonly("packets_sent", &DriverStats::packets_sent)
        .def_readonly("packets_received", &DriverStats::packets_received)
        .def_readonly("fragments_sent", &DriverStats::fragments_sent)
        .def_readonly("fragments_received", &DriverStats::fragments_received)
        .def_readonly("send_errors", &DriverStats::send_errors)
        .def_readonly("receive_errors", &DriverStats::receive_errors)
        .def_readonly("last_state_time", &DriverStats::last_state_time)
        .def_readonly("clock_sync", &DriverStats::clock_sync,
            "Clock synchronization state")
        .def("__repr__", [](const DriverStats& s) {
            return "<DriverStats tx=" + std::to_string(s.packets_sent) +
                   " rx=" + std::to_string(s.packets_received) +
                   " errors=" + std::to_string(s.send_errors + s.receive_errors) + ">";
        });

    // SimulationState (schema::StatePacketT)
    py::class_<SimulationState>(m, "SimulationState")
        .def(py::init<>())
        .def_readwrite("sequence", &SimulationState::sequence,
            "Packet sequence number")
        .def_readwrite("time", &SimulationState::time,
            "Simulation time in seconds")
        .def_readwrite("energy_potential", &SimulationState::energy_potential,
            "Potential energy")
        .def_readwrite("energy_kinetic", &SimulationState::energy_kinetic,
            "Kinetic energy")
        .def_readwrite("qpos", &SimulationState::qpos,
            "Generalized positions (length = model.nq)")
        .def_readwrite("qvel", &SimulationState::qvel,
            "Generalized velocities (length = model.nv)")
        .def_readwrite("ctrl", &SimulationState::ctrl,
            "Control values (length = model.nu)")
        .def_readwrite("sensordata", &SimulationState::sensordata,
            "Sensor data (length = model.nsensordata)")
        .def_readwrite("step_index", &SimulationState::step_index,
            "Monotonic step counter")
        .def_readwrite("accepted_ctrl_seq", &SimulationState::accepted_ctrl_seq,
            "Sequence of the control packet applied for this state")
        .def_readwrite("device_wall_us", &SimulationState::device_wall_us,
            "Device steady_clock at capture (microseconds)")
        .def_readwrite("timestep_us", &SimulationState::timestep_us,
            "Model timestep in microseconds")
        .def_readwrite("echo_token", &SimulationState::echo_token,
            "Echo of control's echo_token")
        // Force data (opt-in via sendForceData)
        .def_readwrite("qfrc_actuator", &SimulationState::qfrc_actuator,
            "Actuator forces at each joint (length = nv)")
        .def_readwrite("qfrc_bias", &SimulationState::qfrc_bias,
            "Gravity + Coriolis forces at each joint (length = nv)")
        .def_readwrite("cfrc_int", &SimulationState::cfrc_int,
            "Internal body forces (length = nbody * 6)")
        .def_readwrite("cfrc_ext", &SimulationState::cfrc_ext,
            "External body forces (length = nbody * 6)")
        .def_readwrite("ncon", &SimulationState::ncon,
            "Number of active contacts")
        .def_readwrite("contact_pos", &SimulationState::contact_pos,
            "Contact positions in world frame (length = ncon * 3)")
        .def_readwrite("contact_force", &SimulationState::contact_force,
            "Contact normal forces (length = ncon)")
        .def_readwrite("contact_body1", &SimulationState::contact_body1,
            "Contact body 1 IDs (length = ncon)")
        .def_readwrite("contact_body2", &SimulationState::contact_body2,
            "Contact body 2 IDs (length = ncon)")
        .def("__repr__", [](const SimulationState& s) {
            return "<SimulationState time=" + std::to_string(s.time) +
                   " qpos[" + std::to_string(s.qpos.size()) + "]" +
                   " qvel[" + std::to_string(s.qvel.size()) + "]" +
                   " ncon=" + std::to_string(s.ncon) + ">";
        });

    // ExpiryPolicy enum
    py::enum_<imujoco::schema::ExpiryPolicy>(m, "ExpiryPolicy")
        .value("Default", imujoco::schema::ExpiryPolicy::Default,
            "Use simulation-side config default")
        .value("ZeroImmediate", imujoco::schema::ExpiryPolicy::ZeroImmediate,
            "Zero all actuators immediately on expiry")
        .value("ZeroAfterTimeout", imujoco::schema::ExpiryPolicy::ZeroAfterTimeout,
            "Use per-actuator timeout policy")
        .value("HoldLastValid", imujoco::schema::ExpiryPolicy::HoldLastValid,
            "Hold the last successfully applied values");

    // ControlCommand (schema::ControlPacketT)
    py::class_<ControlCommand>(m, "ControlCommand")
        .def(py::init<>())
        .def_readwrite("sequence", &ControlCommand::sequence,
            "Packet sequence number")
        .def_readwrite("ctrl", &ControlCommand::ctrl,
            "Control values for actuators")
        .def_readwrite("host_timestamp_us", &ControlCommand::host_timestamp_us,
            "Host monotonic timestamp in microseconds (0 = apply immediately)")
        .def_readwrite("target_sim_time", &ControlCommand::target_sim_time,
            "Target simulation time (0.0 = use existing pacing)")
        .def_readwrite("echo_token", &ControlCommand::echo_token,
            "Opaque token echoed back in state")
        .def_readwrite("qfrc_applied", &ControlCommand::qfrc_applied,
            "Generalized forces (length = model.nv)")
        .def_readwrite("xfrc_applied", &ControlCommand::xfrc_applied,
            "Cartesian forces on bodies (length = 6*model.nbody)")
        .def_readwrite("mocap_pos", &ControlCommand::mocap_pos,
            "Mocap body positions (length = 3*model.nmocap)")
        .def_readwrite("mocap_quat", &ControlCommand::mocap_quat,
            "Mocap body quaternions (length = 4*model.nmocap)")
        .def_readwrite("expire_at_sim_time", &ControlCommand::expire_at_sim_time,
            "Simulation time after which this control is stale (-1.0 = no expiry)")
        .def_readwrite("expiry_policy", &ControlCommand::expiry_policy,
            "What to do when this control expires (SimTimeGuarded mode)")
        .def("__repr__", [](const ControlCommand& c) {
            return "<ControlCommand ctrl[" + std::to_string(c.ctrl.size()) + "]>";
        });

    // Driver
    py::class_<Driver>(m, "Driver")
        .def(py::init<const DriverConfig&>(), py::arg("config") = DriverConfig(),
            "Create a driver with the given configuration")

        // Connection
        .def("connect", &Driver::Connect,
            "Connect to the simulation. Returns True if successful.")
        .def("disconnect", &Driver::Disconnect,
            "Disconnect from the simulation.",
            py::call_guard<py::gil_scoped_release>())
        .def("is_connected", &Driver::IsConnected,
            "Check if connected to the simulation.")

        // Control - send ControlCommand
        .def("send_control",
            static_cast<void (Driver::*)(const ControlCommand&)>(&Driver::SendControl),
            py::arg("cmd"),
            "Send a control command (non-blocking).",
            py::call_guard<py::gil_scoped_release>())

        // Control - send list of floats
        .def("send_control",
            [](Driver& self, const std::vector<double>& ctrl) {
                self.SendControl(std::span<const double>(ctrl));
            },
            py::arg("ctrl"),
            "Send control values as a list (non-blocking).",
            py::call_guard<py::gil_scoped_release>())

        // Subscribe - with GIL handling for Python callbacks
        .def("subscribe",
            [](Driver& self, py::function callback) {
                // Store in shared_ptr with GIL-aware deleter so lambda copies
                // only touch atomic refcount (no GIL needed), and the Python
                // object is destroyed under the GIL.
                auto shared_cb = std::shared_ptr<py::function>(
                    new py::function(std::move(callback)),
                    [](py::function* fn) {
                        py::gil_scoped_acquire acquire;
                        delete fn;
                    });
                auto wrapped = [shared_cb](const SimulationState& state) {
                    py::gil_scoped_acquire acquire;
                    try {
                        (*shared_cb)(state);
                    } catch (py::error_already_set&) {
                        PyErr_Print();
                        PyErr_Clear();
                    }
                };
                return self.Subscribe(std::move(wrapped));
            },
            py::arg("callback"),
            "Subscribe to state updates. Callback receives SimulationState.\n"
            "Returns subscription ID for unsubscribing.\n"
            "Note: Callback runs on RX thread - keep it lightweight.")

        .def("unsubscribe", &Driver::Unsubscribe, py::arg("subscription_id"),
            "Unsubscribe from state updates. Returns True if found.")

        // Error callback
        .def("on_error",
            [](Driver& self, py::function callback) {
                auto shared_cb = std::shared_ptr<py::function>(
                    new py::function(std::move(callback)),
                    [](py::function* fn) {
                        py::gil_scoped_acquire acquire;
                        delete fn;
                    });
                auto wrapped = [shared_cb](std::error_code ec, const std::string& msg) {
                    py::gil_scoped_acquire acquire;
                    try {
                        (*shared_cb)(ec.value(), msg);
                    } catch (py::error_already_set&) {
                        PyErr_Print();
                        PyErr_Clear();
                    }
                };
                self.OnError(std::move(wrapped));
            },
            py::arg("callback"),
            "Set error callback. Callback receives (error_code: int, message: str).")

        // Receive control
        .def("start_receiving", &Driver::StartReceiving,
            "Start the RX thread for receiving state updates.")
        .def("stop_receiving", &Driver::StopReceiving,
            "Stop the RX thread.")
        .def("is_receiving", &Driver::IsReceiving,
            "Check if RX thread is running.")

        // Stats
        .def("get_stats", &Driver::GetStats,
            "Get current driver statistics.")
        .def("reset_stats", &Driver::ResetStats,
            "Reset statistics to zero.")

        // Time sync
        .def("get_clock_sync", &Driver::GetClockSync,
            "Get current clock synchronization state.")
        .def("predict_sim_time",
            [](const Driver& self) -> py::object {
                auto result = self.PredictSimTime();
                if (result) return py::cast(*result);
                return py::none();
            },
            "Predict current simulation time (None if not synced).")

        // Config
        .def_property_readonly("config", &Driver::Config,
            "Get the driver configuration.")

        .def("__repr__", [](const Driver& d) {
            auto& c = d.Config();
            return "<Driver " + c.host + ":" + std::to_string(c.port) +
                   " connected=" + (d.IsConnected() ? "True" : "False") + ">";
        });
}
