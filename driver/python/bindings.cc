// bindings.cc
// Python bindings for imujoco_driver using pybind11

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "imujoco/driver/driver.h"

namespace py = pybind11;
using namespace imujoco::driver;

PYBIND11_MODULE(_imujoco_driver, m) {
    m.doc() = "iMuJoCo Driver - Thread-safe UDP client for iMuJoCo simulation";

    // DriverConfig
    py::class_<DriverConfig>(m, "DriverConfig")
        .def(py::init<>())
        .def_readwrite("host", &DriverConfig::host,
            "Target simulation host (default: 127.0.0.1)")
        .def_readwrite("port", &DriverConfig::port,
            "Target simulation UDP port (default: 8888)")
        .def_readwrite("local_port", &DriverConfig::local_port,
            "Local bind port (0 = ephemeral)")
        .def_readwrite("timeout_ms", &DriverConfig::timeout_ms,
            "Receive timeout in milliseconds (0 = no timeout)")
        .def_readwrite("auto_start_receiving", &DriverConfig::auto_start_receiving,
            "Auto-start receiving on connect (default: true)")
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
        .def("__repr__", [](const SimulationState& s) {
            return "<SimulationState time=" + std::to_string(s.time) +
                   " qpos[" + std::to_string(s.qpos.size()) + "]" +
                   " qvel[" + std::to_string(s.qvel.size()) + "]>";
        });

    // ControlCommand (schema::ControlPacketT)
    py::class_<ControlCommand>(m, "ControlCommand")
        .def(py::init<>())
        .def_readwrite("sequence", &ControlCommand::sequence,
            "Packet sequence number")
        .def_readwrite("ctrl", &ControlCommand::ctrl,
            "Control values for actuators")
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
            "Disconnect from the simulation.")
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
                // Wrap Python callback to acquire GIL when called from C++ thread
                auto wrapped = [callback = std::move(callback)](const SimulationState& state) {
                    py::gil_scoped_acquire acquire;
                    try {
                        callback(state);
                    } catch (py::error_already_set& e) {
                        // Log Python exception but don't crash
                        py::print("Error in state callback:", e.what());
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
                auto wrapped = [callback = std::move(callback)](std::error_code ec, const std::string& msg) {
                    py::gil_scoped_acquire acquire;
                    try {
                        callback(ec.value(), msg);
                    } catch (py::error_already_set& e) {
                        py::print("Error in error callback:", e.what());
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

        // Config
        .def_property_readonly("config", &Driver::Config,
            "Get the driver configuration.")

        .def("__repr__", [](const Driver& d) {
            auto& c = d.Config();
            return "<Driver " + c.host + ":" + std::to_string(c.port) +
                   " connected=" + (d.IsConnected() ? "True" : "False") + ">";
        });
}
