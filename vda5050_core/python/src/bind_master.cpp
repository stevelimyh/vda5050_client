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

#include "vda5050_core/master/actions/instant_action_assignment_result.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/master/assignment_result.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/master/pose_view.hpp"
#include "vda5050_core/python/register.hpp"

namespace py = pybind11;

namespace vda5050_core::python {

void register_master(py::module_& m)
{
  py::enum_<master::AGVState>(m, "AGVState", py::module_local())
    .value("STATE_UNKNOWN", master::AGVState::STATE_UNKNOWN)
    .value("AVAILABLE", master::AGVState::AVAILABLE)
    .value("UNAVAILABLE", master::AGVState::UNAVAILABLE)
    .value("ERROR", master::AGVState::ERROR);

  // PoseSource::None is exposed as NONE (None is a Python keyword).
  py::enum_<master::PoseSource>(m, "PoseSource", py::module_local())
    .value("NONE", master::PoseSource::None)
    .value("VISUALIZATION", master::PoseSource::Visualization)
    .value("STATE", master::PoseSource::State)
    .value("EXTRAPOLATED", master::PoseSource::Extrapolated);

  py::enum_<master::AssignmentDecision>(
    m, "AssignmentDecision", py::module_local())
    .value("ASSIGNED", master::AssignmentDecision::ASSIGNED)
    .value("AGV_NOT_ONBOARDED", master::AssignmentDecision::AGV_NOT_ONBOARDED)
    .value("AGV_OFFLINE", master::AssignmentDecision::AGV_OFFLINE)
    .value("AGV_NOT_READY", master::AssignmentDecision::AGV_NOT_READY)
    .value("AGV_MODE_NOT_AUTO", master::AssignmentDecision::AGV_MODE_NOT_AUTO)
    .value(
      "AGV_POSITION_NOT_INITIALIZED",
      master::AssignmentDecision::AGV_POSITION_NOT_INITIALIZED)
    .value("AGV_NO_STATE_YET", master::AssignmentDecision::AGV_NO_STATE_YET)
    .value("STITCH_REJECTED", master::AssignmentDecision::STITCH_REJECTED)
    .value("STITCH_QUEUED", master::AssignmentDecision::STITCH_QUEUED);

  py::enum_<master::InstantActionDecision>(
    m, "InstantActionDecision", py::module_local())
    .value("ASSIGNED", master::InstantActionDecision::ASSIGNED)
    .value(
      "AGV_NOT_ONBOARDED", master::InstantActionDecision::AGV_NOT_ONBOARDED)
    .value("AGV_OFFLINE", master::InstantActionDecision::AGV_OFFLINE)
    .value(
      "DUPLICATE_ACTION_ID", master::InstantActionDecision::DUPLICATE_ACTION_ID)
    .value("AGV_QUEUE_FULL", master::InstantActionDecision::AGV_QUEUE_FULL)
    .value(
      "HARD_ACTION_BLOCKED", master::InstantActionDecision::HARD_ACTION_BLOCKED)
    .value(
      "ACTION_BLOCKED_BY_DRIVING",
      master::InstantActionDecision::ACTION_BLOCKED_BY_DRIVING)
    .value(
      "AGV_MODE_NOT_AUTO_FOR_ACTION",
      master::InstantActionDecision::AGV_MODE_NOT_AUTO_FOR_ACTION);

  py::class_<master::AssignmentResult>(
    m, "AssignmentResult", py::module_local())
    .def_readonly("decision", &master::AssignmentResult::decision)
    .def_readonly("errors", &master::AssignmentResult::errors)
    .def("__bool__", [](const master::AssignmentResult& r) {
      return static_cast<bool>(r);
    });

  py::class_<master::InstantActionAssignmentResult>(
    m, "InstantActionAssignmentResult", py::module_local())
    .def_readonly("decision", &master::InstantActionAssignmentResult::decision)
    .def_readonly("errors", &master::InstantActionAssignmentResult::errors)
    .def("__bool__", [](const master::InstantActionAssignmentResult& r) {
      return static_cast<bool>(r);
    });

  py::class_<master::VDA5050Master::BrokerStatusSnapshot>(
    m, "BrokerStatusSnapshot", py::module_local())
    .def_readonly(
      "connected", &master::VDA5050Master::BrokerStatusSnapshot::connected)
    .def_readonly(
      "last_disconnect_at",
      &master::VDA5050Master::BrokerStatusSnapshot::last_disconnect_at)
    .def_readonly(
      "reconnect_count",
      &master::VDA5050Master::BrokerStatusSnapshot::reconnect_count);

  py::class_<master::PoseView>(m, "PoseView", py::module_local())
    .def_readonly("driving", &master::PoseView::driving)
    .def_readonly("agv_position", &master::PoseView::agv_position)
    .def_readonly("velocity", &master::PoseView::velocity)
    .def_readonly("source", &master::PoseView::source)
    .def_readonly("data_age", &master::PoseView::data_age);

  py::class_<master::ActiveOrderSnapshot>(
    m, "ActiveOrderSnapshot", py::module_local())
    .def_readonly("has_active", &master::ActiveOrderSnapshot::has_active)
    .def_readonly("order_id", &master::ActiveOrderSnapshot::order_id)
    .def_readonly(
      "order_update_id", &master::ActiveOrderSnapshot::order_update_id)
    .def_readonly("nodes", &master::ActiveOrderSnapshot::nodes)
    .def_readonly("edges", &master::ActiveOrderSnapshot::edges)
    .def_readonly(
      "last_node_sequence_id",
      &master::ActiveOrderSnapshot::last_node_sequence_id)
    .def_readonly(
      "state_order_update_id",
      &master::ActiveOrderSnapshot::state_order_update_id)
    .def_readonly(
      "state_order_id", &master::ActiveOrderSnapshot::state_order_id)
    .def_readonly(
      "order_complete", &master::ActiveOrderSnapshot::order_complete);

  py::class_<master::AGV::OrderStatusBundle>(
    m, "OrderStatusBundle", py::module_local())
    .def_readonly("state", &master::AGV::OrderStatusBundle::state)
    .def_readonly(
      "state_received_at", &master::AGV::OrderStatusBundle::state_received_at)
    .def_readonly(
      "active_order_snapshot",
      &master::AGV::OrderStatusBundle::active_order_snapshot)
    .def_readonly(
      "pending_stitch_count",
      &master::AGV::OrderStatusBundle::pending_stitch_count);

  // factsheet field deferred until the Factsheet type is bound.
  py::class_<master::AGV::StatusSnapshot>(
    m, "StatusSnapshot", py::module_local())
    .def_readonly("state", &master::AGV::StatusSnapshot::state)
    .def_readonly("connection", &master::AGV::StatusSnapshot::connection)
    .def_readonly(
      "state_received_at", &master::AGV::StatusSnapshot::state_received_at)
    .def_readonly(
      "connection_received_at",
      &master::AGV::StatusSnapshot::connection_received_at)
    .def_readonly(
      "factsheet_received_at",
      &master::AGV::StatusSnapshot::factsheet_received_at);
}

}  // namespace vda5050_core::python
