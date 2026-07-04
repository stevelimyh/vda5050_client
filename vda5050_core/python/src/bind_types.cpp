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

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "vda5050_core/python/register.hpp"
#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_parameter.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/bounding_box_reference.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/control_point.hpp"
#include "vda5050_core/types/e_stop.hpp"
#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/edge_state.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/error_level.hpp"
#include "vda5050_core/types/error_reference.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/info.hpp"
#include "vda5050_core/types/info_level.hpp"
#include "vda5050_core/types/info_reference.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/load.hpp"
#include "vda5050_core/types/load_dimensions.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/node_position.hpp"
#include "vda5050_core/types/node_state.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/orientation_type.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/trajectory.hpp"
#include "vda5050_core/types/velocity.hpp"
#include "vda5050_core/types/visualization.hpp"

namespace py = pybind11;

namespace vda5050_core::python {

void register_types(py::module_& m)
{
  py::enum_<types::OperatingMode>(m, "OperatingMode", py::module_local())
    .value("AUTOMATIC", types::OperatingMode::AUTOMATIC)
    .value("SEMIAUTOMATIC", types::OperatingMode::SEMIAUTOMATIC)
    .value("MANUAL", types::OperatingMode::MANUAL)
    .value("SERVICE", types::OperatingMode::SERVICE)
    .value("TEACHIN", types::OperatingMode::TEACHIN);

  py::enum_<types::ActionStatus>(m, "ActionStatus", py::module_local())
    .value("WAITING", types::ActionStatus::WAITING)
    .value("INITIALIZING", types::ActionStatus::INITIALIZING)
    .value("RUNNING", types::ActionStatus::RUNNING)
    .value("PAUSED", types::ActionStatus::PAUSED)
    .value("FINISHED", types::ActionStatus::FINISHED)
    .value("FAILED", types::ActionStatus::FAILED);

  py::enum_<types::EStop>(m, "EStop", py::module_local())
    .value("AUTOACK", types::EStop::AUTOACK)
    .value("MANUAL", types::EStop::MANUAL)
    .value("REMOTE", types::EStop::REMOTE)
    .value("NONE", types::EStop::NONE);

  py::enum_<types::ErrorLevel>(m, "ErrorLevel", py::module_local())
    .value("WARNING", types::ErrorLevel::WARNING)
    .value("FATAL", types::ErrorLevel::FATAL);

  py::enum_<types::InfoLevel>(m, "InfoLevel", py::module_local())
    .value("DEBUG", types::InfoLevel::DEBUG)
    .value("INFO", types::InfoLevel::INFO);

  py::enum_<types::ConnectionState>(m, "ConnectionState", py::module_local())
    .value("ONLINE", types::ConnectionState::ONLINE)
    .value("OFFLINE", types::ConnectionState::OFFLINE)
    .value("CONNECTIONBROKEN", types::ConnectionState::CONNECTIONBROKEN);

  py::enum_<types::OrientationType>(m, "OrientationType", py::module_local())
    .value("GLOBAL", types::OrientationType::GLOBAL)
    .value("TANGENTIAL", types::OrientationType::TANGENTIAL);

  py::class_<types::Header>(m, "Header", py::module_local())
    .def(py::init<>())
    .def_readwrite("header_id", &types::Header::header_id)
    .def_readwrite("timestamp", &types::Header::timestamp)
    .def_readwrite("version", &types::Header::version)
    .def_readwrite("manufacturer", &types::Header::manufacturer)
    .def_readwrite("serial_number", &types::Header::serial_number)
    .def("__eq__", &types::Header::operator==)
    .def("__ne__", &types::Header::operator!=);

  py::class_<types::ErrorReference>(m, "ErrorReference", py::module_local())
    .def(py::init<>())
    .def_readwrite("reference_key", &types::ErrorReference::reference_key)
    .def_readwrite("reference_value", &types::ErrorReference::reference_value)
    .def("__eq__", &types::ErrorReference::operator==)
    .def("__ne__", &types::ErrorReference::operator!=);

  py::class_<types::Error>(m, "Error", py::module_local())
    .def(py::init<>())
    .def_readwrite("error_type", &types::Error::error_type)
    .def_readwrite("error_references", &types::Error::error_references)
    .def_readwrite("error_description", &types::Error::error_description)
    .def_readwrite("error_level", &types::Error::error_level)
    .def("__eq__", &types::Error::operator==)
    .def("__ne__", &types::Error::operator!=);

  py::class_<types::InfoReference>(m, "InfoReference", py::module_local())
    .def(py::init<>())
    .def_readwrite("reference_key", &types::InfoReference::reference_key)
    .def_readwrite("reference_value", &types::InfoReference::reference_value)
    .def("__eq__", &types::InfoReference::operator==)
    .def("__ne__", &types::InfoReference::operator!=);

  py::class_<types::Info>(m, "Info", py::module_local())
    .def(py::init<>())
    .def_readwrite("info_type", &types::Info::info_type)
    .def_readwrite("info_references", &types::Info::info_references)
    .def_readwrite("info_description", &types::Info::info_description)
    .def_readwrite("info_level", &types::Info::info_level)
    .def("__eq__", &types::Info::operator==)
    .def("__ne__", &types::Info::operator!=);

  py::class_<types::AGVPosition>(m, "AGVPosition", py::module_local())
    .def(py::init<>())
    .def_readwrite("x", &types::AGVPosition::x)
    .def_readwrite("y", &types::AGVPosition::y)
    .def_readwrite("theta", &types::AGVPosition::theta)
    .def_readwrite(
      "position_initialized", &types::AGVPosition::position_initialized)
    .def_readwrite("map_id", &types::AGVPosition::map_id)
    .def_readwrite("map_description", &types::AGVPosition::map_description)
    .def_readwrite(
      "localization_score", &types::AGVPosition::localization_score)
    .def_readwrite("deviation_range", &types::AGVPosition::deviation_range)
    .def("__eq__", &types::AGVPosition::operator==)
    .def("__ne__", &types::AGVPosition::operator!=);

  py::class_<types::Velocity>(m, "Velocity", py::module_local())
    .def(py::init<>())
    .def_readwrite("vx", &types::Velocity::vx)
    .def_readwrite("vy", &types::Velocity::vy)
    .def_readwrite("omega", &types::Velocity::omega)
    .def("__eq__", &types::Velocity::operator==)
    .def("__ne__", &types::Velocity::operator!=);

  py::class_<types::BatteryState>(m, "BatteryState", py::module_local())
    .def(py::init<>())
    .def_readwrite("battery_charge", &types::BatteryState::battery_charge)
    .def_readwrite("battery_voltage", &types::BatteryState::battery_voltage)
    .def_readwrite("battery_health", &types::BatteryState::battery_health)
    .def_readwrite("charging", &types::BatteryState::charging)
    .def_readwrite("reach", &types::BatteryState::reach)
    .def("__eq__", &types::BatteryState::operator==)
    .def("__ne__", &types::BatteryState::operator!=);

  py::class_<types::SafetyState>(m, "SafetyState", py::module_local())
    .def(py::init<>())
    .def_readwrite("e_stop", &types::SafetyState::e_stop)
    .def_readwrite("field_violation", &types::SafetyState::field_violation)
    .def("__eq__", &types::SafetyState::operator==)
    .def("__ne__", &types::SafetyState::operator!=);

  py::class_<types::ActionState>(m, "ActionState", py::module_local())
    .def(py::init<>())
    .def_readwrite("action_id", &types::ActionState::action_id)
    .def_readwrite("action_type", &types::ActionState::action_type)
    .def_readwrite(
      "action_description", &types::ActionState::action_description)
    .def_readwrite("action_status", &types::ActionState::action_status)
    .def_readwrite(
      "result_description", &types::ActionState::result_description)
    .def("__eq__", &types::ActionState::operator==)
    .def("__ne__", &types::ActionState::operator!=);

  py::class_<types::NodePosition>(m, "NodePosition", py::module_local())
    .def(py::init<>())
    .def_readwrite("x", &types::NodePosition::x)
    .def_readwrite("y", &types::NodePosition::y)
    .def_readwrite("theta", &types::NodePosition::theta)
    .def_readwrite(
      "allowed_deviation_x_y", &types::NodePosition::allowed_deviation_x_y)
    .def_readwrite(
      "allowed_deviation_theta", &types::NodePosition::allowed_deviation_theta)
    .def_readwrite("map_id", &types::NodePosition::map_id)
    .def_readwrite("map_description", &types::NodePosition::map_description)
    .def("__eq__", &types::NodePosition::operator==)
    .def("__ne__", &types::NodePosition::operator!=);

  py::class_<types::NodeState>(m, "NodeState", py::module_local())
    .def(py::init<>())
    .def_readwrite("node_id", &types::NodeState::node_id)
    .def_readwrite("sequence_id", &types::NodeState::sequence_id)
    .def_readwrite("node_description", &types::NodeState::node_description)
    .def_readwrite("node_position", &types::NodeState::node_position)
    .def_readwrite("released", &types::NodeState::released)
    .def("__eq__", &types::NodeState::operator==)
    .def("__ne__", &types::NodeState::operator!=);

  py::class_<types::ControlPoint>(m, "ControlPoint", py::module_local())
    .def(py::init<>())
    .def_readwrite("x", &types::ControlPoint::x)
    .def_readwrite("y", &types::ControlPoint::y)
    .def_readwrite("weight", &types::ControlPoint::weight)
    .def("__eq__", &types::ControlPoint::operator==)
    .def("__ne__", &types::ControlPoint::operator!=);

  py::class_<types::Trajectory>(m, "Trajectory", py::module_local())
    .def(py::init<>())
    .def_readwrite("degree", &types::Trajectory::degree)
    .def_readwrite("knot_vector", &types::Trajectory::knot_vector)
    .def_readwrite("control_points", &types::Trajectory::control_points)
    .def("__eq__", &types::Trajectory::operator==)
    .def("__ne__", &types::Trajectory::operator!=);

  py::class_<types::EdgeState>(m, "EdgeState", py::module_local())
    .def(py::init<>())
    .def_readwrite("edge_id", &types::EdgeState::edge_id)
    .def_readwrite("sequence_id", &types::EdgeState::sequence_id)
    .def_readwrite("edge_description", &types::EdgeState::edge_description)
    .def_readwrite("released", &types::EdgeState::released)
    .def_readwrite("trajectory", &types::EdgeState::trajectory)
    .def("__eq__", &types::EdgeState::operator==)
    .def("__ne__", &types::EdgeState::operator!=);

  py::class_<types::BoundingBoxReference>(
    m, "BoundingBoxReference", py::module_local())
    .def(py::init<>())
    .def_readwrite("x", &types::BoundingBoxReference::x)
    .def_readwrite("y", &types::BoundingBoxReference::y)
    .def_readwrite("z", &types::BoundingBoxReference::z)
    .def_readwrite("theta", &types::BoundingBoxReference::theta)
    .def("__eq__", &types::BoundingBoxReference::operator==)
    .def("__ne__", &types::BoundingBoxReference::operator!=);

  py::class_<types::LoadDimensions>(m, "LoadDimensions", py::module_local())
    .def(py::init<>())
    .def_readwrite("length", &types::LoadDimensions::length)
    .def_readwrite("width", &types::LoadDimensions::width)
    .def_readwrite("height", &types::LoadDimensions::height)
    .def("__eq__", &types::LoadDimensions::operator==)
    .def("__ne__", &types::LoadDimensions::operator!=);

  py::class_<types::Load>(m, "Load", py::module_local())
    .def(py::init<>())
    .def_readwrite("load_id", &types::Load::load_id)
    .def_readwrite("load_type", &types::Load::load_type)
    .def_readwrite("load_position", &types::Load::load_position)
    .def_readwrite(
      "bounding_box_reference", &types::Load::bounding_box_reference)
    .def_readwrite("load_dimensions", &types::Load::load_dimensions)
    .def_readwrite("weight", &types::Load::weight)
    .def("__eq__", &types::Load::operator==)
    .def("__ne__", &types::Load::operator!=);

  py::class_<types::State>(m, "State", py::module_local())
    .def(py::init<>())
    .def_readwrite("header", &types::State::header)
    .def_readwrite("order_id", &types::State::order_id)
    .def_readwrite("order_update_id", &types::State::order_update_id)
    .def_readwrite("zone_set_id", &types::State::zone_set_id)
    .def_readwrite("last_node_id", &types::State::last_node_id)
    .def_readwrite(
      "last_node_sequence_id", &types::State::last_node_sequence_id)
    .def_readwrite("node_states", &types::State::node_states)
    .def_readwrite("edge_states", &types::State::edge_states)
    .def_readwrite("agv_position", &types::State::agv_position)
    .def_readwrite("velocity", &types::State::velocity)
    .def_readwrite("loads", &types::State::loads)
    .def_readwrite("driving", &types::State::driving)
    .def_readwrite("paused", &types::State::paused)
    .def_readwrite("new_base_request", &types::State::new_base_request)
    .def_readwrite(
      "distance_since_last_node", &types::State::distance_since_last_node)
    .def_readwrite("action_states", &types::State::action_states)
    .def_readwrite("battery_state", &types::State::battery_state)
    .def_readwrite("operating_mode", &types::State::operating_mode)
    .def_readwrite("errors", &types::State::errors)
    .def_readwrite("information", &types::State::information)
    .def_readwrite("safety_state", &types::State::safety_state)
    .def("__eq__", &types::State::operator==)
    .def("__ne__", &types::State::operator!=);

  py::class_<types::Connection>(m, "Connection", py::module_local())
    .def(py::init<>())
    .def_readwrite("header", &types::Connection::header)
    .def_readwrite("connection_state", &types::Connection::connection_state)
    .def("__eq__", &types::Connection::operator==)
    .def("__ne__", &types::Connection::operator!=);

  py::class_<types::Visualization>(m, "Visualization", py::module_local())
    .def(py::init<>())
    .def_readwrite("header", &types::Visualization::header)
    .def_readwrite("agv_position", &types::Visualization::agv_position)
    .def_readwrite("velocity", &types::Visualization::velocity)
    .def("__eq__", &types::Visualization::operator==)
    .def("__ne__", &types::Visualization::operator!=);

  py::enum_<types::BlockingType>(m, "BlockingType", py::module_local())
    .value("NONE", types::BlockingType::NONE)
    .value("SOFT", types::BlockingType::SOFT)
    .value("HARD", types::BlockingType::HARD);

  py::class_<types::ActionParameter>(m, "ActionParameter", py::module_local())
    .def(py::init<>())
    .def_readwrite("key", &types::ActionParameter::key)
    .def_readwrite("value", &types::ActionParameter::value)
    .def("__eq__", &types::ActionParameter::operator==)
    .def("__ne__", &types::ActionParameter::operator!=);

  py::class_<types::Action>(m, "Action", py::module_local())
    .def(py::init<>())
    .def_readwrite("action_type", &types::Action::action_type)
    .def_readwrite("action_id", &types::Action::action_id)
    .def_readwrite("blocking_type", &types::Action::blocking_type)
    .def_readwrite("action_description", &types::Action::action_description)
    .def_readwrite("action_parameters", &types::Action::action_parameters)
    .def("__eq__", &types::Action::operator==)
    .def("__ne__", &types::Action::operator!=);

  py::class_<types::Node>(m, "Node", py::module_local())
    .def(py::init<>())
    .def_readwrite("node_id", &types::Node::node_id)
    .def_readwrite("sequence_id", &types::Node::sequence_id)
    .def_readwrite("released", &types::Node::released)
    .def_readwrite("actions", &types::Node::actions)
    .def_readwrite("node_position", &types::Node::node_position)
    .def_readwrite("node_description", &types::Node::node_description)
    .def("__eq__", &types::Node::operator==)
    .def("__ne__", &types::Node::operator!=);

  py::class_<types::Edge>(m, "Edge", py::module_local())
    .def(py::init<>())
    .def_readwrite("edge_id", &types::Edge::edge_id)
    .def_readwrite("sequence_id", &types::Edge::sequence_id)
    .def_readwrite("start_node_id", &types::Edge::start_node_id)
    .def_readwrite("end_node_id", &types::Edge::end_node_id)
    .def_readwrite("released", &types::Edge::released)
    .def_readwrite("actions", &types::Edge::actions)
    .def_readwrite("edge_description", &types::Edge::edge_description)
    .def_readwrite("max_speed", &types::Edge::max_speed)
    .def_readwrite("max_height", &types::Edge::max_height)
    .def_readwrite("min_height", &types::Edge::min_height)
    .def_readwrite("orientation", &types::Edge::orientation)
    .def_readwrite("orientation_type", &types::Edge::orientation_type)
    .def_readwrite("direction", &types::Edge::direction)
    .def_readwrite("rotation_allowed", &types::Edge::rotation_allowed)
    .def_readwrite("max_rotation_speed", &types::Edge::max_rotation_speed)
    .def_readwrite("trajectory", &types::Edge::trajectory)
    .def_readwrite("length", &types::Edge::length)
    .def("__eq__", &types::Edge::operator==)
    .def("__ne__", &types::Edge::operator!=);

  py::class_<types::Order>(m, "Order", py::module_local())
    .def(py::init<>())
    .def_readwrite("header", &types::Order::header)
    .def_readwrite("order_id", &types::Order::order_id)
    .def_readwrite("order_update_id", &types::Order::order_update_id)
    .def_readwrite("nodes", &types::Order::nodes)
    .def_readwrite("edges", &types::Order::edges)
    .def_readwrite("zone_set_id", &types::Order::zone_set_id)
    .def("__eq__", &types::Order::operator==)
    .def("__ne__", &types::Order::operator!=);

  py::class_<types::InstantActions>(m, "InstantActions", py::module_local())
    .def(py::init<>())
    .def_readwrite("header", &types::InstantActions::header)
    .def_readwrite("actions", &types::InstantActions::actions)
    .def("__eq__", &types::InstantActions::operator==)
    .def("__ne__", &types::InstantActions::operator!=);
}

}  // namespace vda5050_core::python
