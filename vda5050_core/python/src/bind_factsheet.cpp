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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "vda5050_core/python/register.hpp"
#include "vda5050_core/types/action_parameter_factsheet.hpp"
#include "vda5050_core/types/action_scope.hpp"
#include "vda5050_core/types/agv_action.hpp"
#include "vda5050_core/types/agv_class.hpp"
#include "vda5050_core/types/agv_geometry.hpp"
#include "vda5050_core/types/agv_kinematic.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/bounding_box_reference.hpp"
#include "vda5050_core/types/envelope2d.hpp"
#include "vda5050_core/types/envelope3d.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/load_dimensions.hpp"
#include "vda5050_core/types/load_set.hpp"
#include "vda5050_core/types/load_specification.hpp"
#include "vda5050_core/types/max_array_lens.hpp"
#include "vda5050_core/types/max_string_lens.hpp"
#include "vda5050_core/types/optional_parameter.hpp"
#include "vda5050_core/types/physical_parameters.hpp"
#include "vda5050_core/types/polygon_point.hpp"
#include "vda5050_core/types/position.hpp"
#include "vda5050_core/types/protocol_features.hpp"
#include "vda5050_core/types/protocol_limits.hpp"
#include "vda5050_core/types/support.hpp"
#include "vda5050_core/types/timing.hpp"
#include "vda5050_core/types/type_specification.hpp"
#include "vda5050_core/types/value_data_type.hpp"
#include "vda5050_core/types/wheel_definition.hpp"

namespace py = pybind11;

namespace vda5050_core::python {

void register_factsheet(py::module_& m)
{
  py::enum_<types::AGVKinematic>(m, "AGVKinematic", py::module_local())
    .value("DIFF", types::AGVKinematic::DIFF)
    .value("OMNI", types::AGVKinematic::OMNI)
    .value("THREEWHEEL", types::AGVKinematic::THREEWHEEL);

  py::enum_<types::AGVClass>(m, "AGVClass", py::module_local())
    .value("FORKLIFT", types::AGVClass::FORKLIFT)
    .value("CONVEYOR", types::AGVClass::CONVEYOR)
    .value("TUGGER", types::AGVClass::TUGGER)
    .value("CARRIER", types::AGVClass::CARRIER);

  py::enum_<types::Support>(m, "Support", py::module_local())
    .value("SUPPORTED", types::Support::SUPPORTED)
    .value("REQUIRED", types::Support::REQUIRED);

  py::enum_<types::ActionScope>(m, "ActionScope", py::module_local())
    .value("INSTANT", types::ActionScope::INSTANT)
    .value("NODE", types::ActionScope::NODE)
    .value("EDGE", types::ActionScope::EDGE);

  py::enum_<types::ValueDataType>(m, "ValueDataType", py::module_local())
    .value("BOOL", types::ValueDataType::BOOL)
    .value("NUMBER", types::ValueDataType::NUMBER)
    .value("INTEGER", types::ValueDataType::INTEGER)
    .value("FLOAT", types::ValueDataType::FLOAT)
    .value("STRING", types::ValueDataType::STRING)
    .value("OBJECT", types::ValueDataType::OBJECT)
    .value("ARRAY", types::ValueDataType::ARRAY);

  py::enum_<types::WheelDefinitionType>(
    m, "WheelDefinitionType", py::module_local())
    .value("DRIVE", types::WheelDefinitionType::DRIVE)
    .value("CASTER", types::WheelDefinitionType::CASTER)
    .value("FIXED", types::WheelDefinitionType::FIXED)
    .value("MECANUM", types::WheelDefinitionType::MECANUM);

  py::class_<types::Position>(m, "Position", py::module_local())
    .def(py::init<>())
    .def_readwrite("x", &types::Position::x)
    .def_readwrite("y", &types::Position::y)
    .def_readwrite("theta", &types::Position::theta)
    .def("__eq__", &types::Position::operator==)
    .def("__ne__", &types::Position::operator!=);

  py::class_<types::PolygonPoint>(m, "PolygonPoint", py::module_local())
    .def(py::init<>())
    .def_readwrite("x", &types::PolygonPoint::x)
    .def_readwrite("y", &types::PolygonPoint::y)
    .def("__eq__", &types::PolygonPoint::operator==)
    .def("__ne__", &types::PolygonPoint::operator!=);

  py::class_<types::TypeSpecification>(
    m, "TypeSpecification", py::module_local())
    .def(py::init<>())
    .def_readwrite("series_name", &types::TypeSpecification::series_name)
    .def_readwrite("agv_kinematic", &types::TypeSpecification::agv_kinematic)
    .def_readwrite("agv_class", &types::TypeSpecification::agv_class)
    .def_readwrite("max_load_mass", &types::TypeSpecification::max_load_mass)
    .def_readwrite(
      "localization_types", &types::TypeSpecification::localization_types)
    .def_readwrite(
      "navigation_types", &types::TypeSpecification::navigation_types)
    .def_readwrite(
      "series_description", &types::TypeSpecification::series_description)
    .def("__eq__", &types::TypeSpecification::operator==)
    .def("__ne__", &types::TypeSpecification::operator!=);

  py::class_<types::PhysicalParameters>(
    m, "PhysicalParameters", py::module_local())
    .def(py::init<>())
    .def_readwrite("speed_min", &types::PhysicalParameters::speed_min)
    .def_readwrite("speed_max", &types::PhysicalParameters::speed_max)
    .def_readwrite(
      "acceleration_max", &types::PhysicalParameters::acceleration_max)
    .def_readwrite(
      "deceleration_max", &types::PhysicalParameters::deceleration_max)
    .def_readwrite("height_min", &types::PhysicalParameters::height_min)
    .def_readwrite("height_max", &types::PhysicalParameters::height_max)
    .def_readwrite("width", &types::PhysicalParameters::width)
    .def_readwrite("length", &types::PhysicalParameters::length)
    .def_readwrite(
      "angular_speed_min", &types::PhysicalParameters::angular_speed_min)
    .def_readwrite(
      "angular_speed_max", &types::PhysicalParameters::angular_speed_max)
    .def("__eq__", &types::PhysicalParameters::operator==)
    .def("__ne__", &types::PhysicalParameters::operator!=);

  py::class_<types::MaxStringLens>(m, "MaxStringLens", py::module_local())
    .def(py::init<>())
    .def_readwrite("msg_len", &types::MaxStringLens::msg_len)
    .def_readwrite("topic_serial_len", &types::MaxStringLens::topic_serial_len)
    .def_readwrite("topic_elem_len", &types::MaxStringLens::topic_elem_len)
    .def_readwrite("id_len", &types::MaxStringLens::id_len)
    .def_readwrite("enum_len", &types::MaxStringLens::enum_len)
    .def_readwrite("load_id_len", &types::MaxStringLens::load_id_len)
    .def_readwrite(
      "id_numerical_only", &types::MaxStringLens::id_numerical_only)
    .def("__eq__", &types::MaxStringLens::operator==)
    .def("__ne__", &types::MaxStringLens::operator!=);

  py::class_<types::MaxArrayLens>(m, "MaxArrayLens", py::module_local())
    .def(py::init<>())
    .def_readwrite("order_nodes", &types::MaxArrayLens::order_nodes)
    .def_readwrite("order_edges", &types::MaxArrayLens::order_edges)
    .def_readwrite("node_actions", &types::MaxArrayLens::node_actions)
    .def_readwrite("edge_actions", &types::MaxArrayLens::edge_actions)
    .def_readwrite(
      "actions_actions_parameters",
      &types::MaxArrayLens::actions_actions_parameters)
    .def_readwrite("instant_actions", &types::MaxArrayLens::instant_actions)
    .def_readwrite(
      "trajectory_knot_vector", &types::MaxArrayLens::trajectory_knot_vector)
    .def_readwrite(
      "trajectory_control_points",
      &types::MaxArrayLens::trajectory_control_points)
    .def_readwrite("state_node_states", &types::MaxArrayLens::state_node_states)
    .def_readwrite("state_edge_states", &types::MaxArrayLens::state_edge_states)
    .def_readwrite("state_loads", &types::MaxArrayLens::state_loads)
    .def_readwrite(
      "state_action_states", &types::MaxArrayLens::state_action_states)
    .def_readwrite("state_errors", &types::MaxArrayLens::state_errors)
    .def_readwrite("state_information", &types::MaxArrayLens::state_information)
    .def_readwrite(
      "error_error_references", &types::MaxArrayLens::error_error_references)
    .def_readwrite(
      "information_info_references",
      &types::MaxArrayLens::information_info_references)
    .def("__eq__", &types::MaxArrayLens::operator==)
    .def("__ne__", &types::MaxArrayLens::operator!=);

  py::class_<types::Timing>(m, "Timing", py::module_local())
    .def(py::init<>())
    .def_readwrite("min_order_interval", &types::Timing::min_order_interval)
    .def_readwrite("min_state_interval", &types::Timing::min_state_interval)
    .def_readwrite(
      "default_state_interval", &types::Timing::default_state_interval)
    .def_readwrite(
      "visualization_interval", &types::Timing::visualization_interval)
    .def("__eq__", &types::Timing::operator==)
    .def("__ne__", &types::Timing::operator!=);

  py::class_<types::ProtocolLimits>(m, "ProtocolLimits", py::module_local())
    .def(py::init<>())
    .def_readwrite("max_string_lens", &types::ProtocolLimits::max_string_lens)
    .def_readwrite("max_array_lens", &types::ProtocolLimits::max_array_lens)
    .def_readwrite("timing", &types::ProtocolLimits::timing)
    .def("__eq__", &types::ProtocolLimits::operator==)
    .def("__ne__", &types::ProtocolLimits::operator!=);

  py::class_<types::OptionalParameter>(
    m, "OptionalParameter", py::module_local())
    .def(py::init<>())
    .def_readwrite("parameter", &types::OptionalParameter::parameter)
    .def_readwrite("support", &types::OptionalParameter::support)
    .def_readwrite("description", &types::OptionalParameter::description)
    .def("__eq__", &types::OptionalParameter::operator==)
    .def("__ne__", &types::OptionalParameter::operator!=);

  py::class_<types::ActionParameterFactsheet>(
    m, "ActionParameterFactsheet", py::module_local())
    .def(py::init<>())
    .def_readwrite("key", &types::ActionParameterFactsheet::key)
    .def_readwrite(
      "value_data_type", &types::ActionParameterFactsheet::value_data_type)
    .def_readwrite("description", &types::ActionParameterFactsheet::description)
    .def_readwrite("is_optional", &types::ActionParameterFactsheet::is_optional)
    .def("__eq__", &types::ActionParameterFactsheet::operator==)
    .def("__ne__", &types::ActionParameterFactsheet::operator!=);

  py::class_<types::AGVAction>(m, "AGVAction", py::module_local())
    .def(py::init<>())
    .def_readwrite("action_type", &types::AGVAction::action_type)
    .def_readwrite("action_scopes", &types::AGVAction::action_scopes)
    .def_readwrite("action_parameters", &types::AGVAction::action_parameters)
    .def_readwrite("result_description", &types::AGVAction::result_description)
    .def_readwrite("action_description", &types::AGVAction::action_description)
    .def_readwrite("blocking_types", &types::AGVAction::blocking_types)
    .def("__eq__", &types::AGVAction::operator==)
    .def("__ne__", &types::AGVAction::operator!=);

  py::class_<types::ProtocolFeatures>(m, "ProtocolFeatures", py::module_local())
    .def(py::init<>())
    .def_readwrite(
      "optional_parameters", &types::ProtocolFeatures::optional_parameters)
    .def_readwrite("agv_actions", &types::ProtocolFeatures::agv_actions)
    .def("__eq__", &types::ProtocolFeatures::operator==)
    .def("__ne__", &types::ProtocolFeatures::operator!=);

  py::class_<types::WheelDefinition>(m, "WheelDefinition", py::module_local())
    .def(py::init<>())
    .def_readwrite("type", &types::WheelDefinition::type)
    .def_readwrite(
      "is_active_driven", &types::WheelDefinition::is_active_driven)
    .def_readwrite(
      "is_active_steered", &types::WheelDefinition::is_active_steered)
    .def_readwrite("position", &types::WheelDefinition::position)
    .def_readwrite("diameter", &types::WheelDefinition::diameter)
    .def_readwrite("width", &types::WheelDefinition::width)
    .def_readwrite(
      "center_displacement", &types::WheelDefinition::center_displacement)
    .def_readwrite("constraints", &types::WheelDefinition::constraints)
    .def("__eq__", &types::WheelDefinition::operator==)
    .def("__ne__", &types::WheelDefinition::operator!=);

  py::class_<types::Envelope2d>(m, "Envelope2d", py::module_local())
    .def(py::init<>())
    .def_readwrite("set", &types::Envelope2d::set)
    .def_readwrite("polygon_points", &types::Envelope2d::polygon_points)
    .def_readwrite("description", &types::Envelope2d::description)
    .def("__eq__", &types::Envelope2d::operator==)
    .def("__ne__", &types::Envelope2d::operator!=);

  py::class_<types::Envelope3d>(m, "Envelope3d", py::module_local())
    .def(py::init<>())
    .def_readwrite("set", &types::Envelope3d::set)
    .def_readwrite("format", &types::Envelope3d::format)
    .def_readwrite("data", &types::Envelope3d::data)
    .def_readwrite("url", &types::Envelope3d::url)
    .def_readwrite("description", &types::Envelope3d::description)
    .def("__eq__", &types::Envelope3d::operator==)
    .def("__ne__", &types::Envelope3d::operator!=);

  py::class_<types::AGVGeometry>(m, "AGVGeometry", py::module_local())
    .def(py::init<>())
    .def_readwrite("wheel_definitions", &types::AGVGeometry::wheel_definitions)
    .def_readwrite("envelopes2d", &types::AGVGeometry::envelopes2d)
    .def_readwrite("envelopes3d", &types::AGVGeometry::envelopes3d)
    .def("__eq__", &types::AGVGeometry::operator==)
    .def("__ne__", &types::AGVGeometry::operator!=);

  py::class_<types::LoadSet>(m, "LoadSet", py::module_local())
    .def(py::init<>())
    .def_readwrite("set_name", &types::LoadSet::set_name)
    .def_readwrite("load_type", &types::LoadSet::load_type)
    .def_readwrite("load_positions", &types::LoadSet::load_positions)
    .def_readwrite(
      "bounding_box_reference", &types::LoadSet::bounding_box_reference)
    .def_readwrite("load_dimensions", &types::LoadSet::load_dimensions)
    .def_readwrite("max_weight", &types::LoadSet::max_weight)
    .def_readwrite(
      "min_load_handling_height", &types::LoadSet::min_load_handling_height)
    .def_readwrite(
      "max_load_handling_height", &types::LoadSet::max_load_handling_height)
    .def_readwrite(
      "min_load_handling_depth", &types::LoadSet::min_load_handling_depth)
    .def_readwrite(
      "max_load_handling_depth", &types::LoadSet::max_load_handling_depth)
    .def_readwrite(
      "min_load_handling_tilt", &types::LoadSet::min_load_handling_tilt)
    .def_readwrite(
      "max_load_handling_tilt", &types::LoadSet::max_load_handling_tilt)
    .def_readwrite("agv_speed_limit", &types::LoadSet::agv_speed_limit)
    .def_readwrite(
      "agv_acceleration_limit", &types::LoadSet::agv_acceleration_limit)
    .def_readwrite(
      "agv_deceleration_limit", &types::LoadSet::agv_deceleration_limit)
    .def_readwrite("pick_time", &types::LoadSet::pick_time)
    .def_readwrite("drop_time", &types::LoadSet::drop_time)
    .def_readwrite("description", &types::LoadSet::description)
    .def("__eq__", &types::LoadSet::operator==)
    .def("__ne__", &types::LoadSet::operator!=);

  py::class_<types::LoadSpecification>(
    m, "LoadSpecification", py::module_local())
    .def(py::init<>())
    .def_readwrite("load_positions", &types::LoadSpecification::load_positions)
    .def_readwrite("load_sets", &types::LoadSpecification::load_sets)
    .def("__eq__", &types::LoadSpecification::operator==)
    .def("__ne__", &types::LoadSpecification::operator!=);

  py::class_<types::Factsheet>(m, "Factsheet", py::module_local())
    .def(py::init<>())
    .def_readwrite("header", &types::Factsheet::header)
    .def_readwrite("type_specification", &types::Factsheet::type_specification)
    .def_readwrite(
      "physical_parameters", &types::Factsheet::physical_parameters)
    .def_readwrite("protocol_limits", &types::Factsheet::protocol_limits)
    .def_readwrite("protocol_features", &types::Factsheet::protocol_features)
    .def_readwrite("agv_geometry", &types::Factsheet::agv_geometry)
    .def_readwrite("load_specification", &types::Factsheet::load_specification)
    .def_readwrite(
      "localization_parameters", &types::Factsheet::localization_parameters)
    .def("__eq__", &types::Factsheet::operator==)
    .def("__ne__", &types::Factsheet::operator!=);
}

}  // namespace vda5050_core::python
