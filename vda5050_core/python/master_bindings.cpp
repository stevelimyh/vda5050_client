/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Draft-standalone module; converges into #71's vda5050_core_python once it
// merges. Shared vda5050_core::types are module_local so the two modules can be
// imported together until then.

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <memory>
#include <string>

#include "vda5050_core/python/master_callbacks.hpp"
#include "vda5050_core/python/register.hpp"

namespace py = pybind11;
namespace pym = vda5050_core::python::master;
namespace vm = vda5050_core::master;

PYBIND11_MODULE(vda5050_master_python, m)
{
  m.doc() = "VDA5050 fleet-master Python bindings";

  auto m_types = m.def_submodule("types", "VDA5050 message types and enums");
  vda5050_core::python::register_types(m_types);
  vda5050_core::python::register_factsheet(m_types);

  auto m_master = m.def_submodule("master", "VDA5050 fleet master API");
  vda5050_core::python::register_master(m_master);

  py::class_<pym::PyMaster, std::shared_ptr<pym::PyMaster>>(
    m_master, "Master", py::module_local())
    .def(
      py::init(&pym::PyMaster::create), py::arg("broker_address"),
      py::arg("client_id"))
    .def_readwrite("on_state", &pym::PyMaster::on_state_cb)
    .def_readwrite("on_connection", &pym::PyMaster::on_connection_cb)
    .def_readwrite("on_visualization", &pym::PyMaster::on_visualization_cb)
    .def_readwrite("on_factsheet", &pym::PyMaster::on_factsheet_cb)
    .def_readwrite("on_node_reached", &pym::PyMaster::on_node_reached_cb)
    .def_readwrite("on_errors_appeared", &pym::PyMaster::on_errors_appeared_cb)
    .def_readwrite("on_errors_resolved", &pym::PyMaster::on_errors_resolved_cb)
    .def_readwrite(
      "on_new_base_requested", &pym::PyMaster::on_new_base_requested_cb)
    .def_readwrite("on_mode_changed", &pym::PyMaster::on_mode_changed_cb)
    .def_readwrite("on_paused", &pym::PyMaster::on_paused_cb)
    .def_readwrite("on_driving", &pym::PyMaster::on_driving_cb)
    .def_readwrite("on_connect", &pym::PyMaster::on_connect_cb)
    .def_readwrite("on_offline", &pym::PyMaster::on_offline_cb)
    .def_readwrite(
      "on_connection_broken", &pym::PyMaster::on_connection_broken_cb)
    .def_readwrite("on_state_timeout", &pym::PyMaster::on_state_timeout_cb)
    .def_readwrite("on_state_resumed", &pym::PyMaster::on_state_resumed_cb)
    .def_readwrite(
      "on_broker_disconnected", &pym::PyMaster::on_broker_disconnected_cb)
    .def_readwrite(
      "on_broker_reconnected", &pym::PyMaster::on_broker_reconnected_cb)
    .def_readwrite("on_loads_changed", &pym::PyMaster::on_loads_changed_cb)
    .def(
      "connect", &vm::VDA5050Master::connect,
      py::call_guard<py::gil_scoped_release>())
    .def(
      "disconnect", &vm::VDA5050Master::disconnect,
      py::call_guard<py::gil_scoped_release>())
    .def("is_connected", &vm::VDA5050Master::is_connected)
    .def("get_broker_status", &vm::VDA5050Master::get_broker_status)
    .def(
      "onboard_agv",
      py::overload_cast<const std::string&, const std::string&, size_t, bool>(
        &vm::VDA5050Master::onboard_agv),
      py::arg("manufacturer"), py::arg("serial_number"),
      py::arg("max_queue_size") = 10, py::arg("drop_oldest") = true,
      py::call_guard<py::gil_scoped_release>())
    .def(
      "onboard_agv",
      py::overload_cast<
        const std::string&, const std::string&, const std::string&, size_t,
        bool>(&vm::VDA5050Master::onboard_agv),
      py::arg("interface_name"), py::arg("manufacturer"),
      py::arg("serial_number"), py::arg("max_queue_size") = 10,
      py::arg("drop_oldest") = true, py::call_guard<py::gil_scoped_release>())
    .def(
      "onboard_agv_batch", &vm::VDA5050Master::onboard_agv_batch,
      py::arg("specs"), py::call_guard<py::gil_scoped_release>())
    .def(
      "offboard_agv", &vm::VDA5050Master::offboard_agv, py::arg("manufacturer"),
      py::arg("serial_number"), py::call_guard<py::gil_scoped_release>())
    .def(
      "offboard_agv_batch", &vm::VDA5050Master::offboard_agv_batch,
      py::arg("keys"), py::call_guard<py::gil_scoped_release>())
    .def(
      "is_agv_onboarded", &vm::VDA5050Master::is_agv_onboarded,
      py::arg("manufacturer"), py::arg("serial_number"))
    .def("get_onboarded_agvs", &vm::VDA5050Master::get_onboarded_agvs)
    .def(
      "publish_order", &vm::VDA5050Master::publish_order,
      py::arg("manufacturer"), py::arg("serial_number"), py::arg("order"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "assign_order", &vm::VDA5050Master::assign_order, py::arg("manufacturer"),
      py::arg("serial_number"), py::arg("order"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "publish_instant_actions", &vm::VDA5050Master::publish_instant_actions,
      py::arg("manufacturer"), py::arg("serial_number"), py::arg("actions"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "assign_instant_actions", &vm::VDA5050Master::assign_instant_actions,
      py::arg("manufacturer"), py::arg("serial_number"), py::arg("actions"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "record_assignment", &vm::VDA5050Master::record_assignment,
      py::arg("manufacturer"), py::arg("serial_number"),
      py::arg("assignment_id"), py::arg("order_id"), py::arg("order_update_id"))
    .def(
      "get_active_assignment_id", &vm::VDA5050Master::get_active_assignment_id,
      py::arg("manufacturer"), py::arg("serial_number"))
    .def(
      "clear_assignment", &vm::VDA5050Master::clear_assignment,
      py::arg("manufacturer"), py::arg("serial_number"))
    .def(
      "get_agv", &vm::VDA5050Master::get_agv, py::arg("manufacturer"),
      py::arg("serial_number"));
}
