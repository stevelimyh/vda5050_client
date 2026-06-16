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

#ifndef VDA5050_MASTER_ROS2__INSTANT_ACTIONS_SEND_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__INSTANT_ACTIONS_SEND_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/actions/instant_action_assignment_result.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_master_ros2/srv/assign_instant_actions.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// InstantActionsSendService (Task #61).
// =============================================================================
//
// Synchronous ROS 2 service that lets an FMS dispatch a VDA5050
// InstantActions message to one AGV. Wraps
// `VDA5050Master::assign_instant_actions(mfg, serial, actions)` —
// inheriting its sync action-conflict detection (#22), §6.10.6 mode
// gate with §6.8.1 allowlist exemption, and DUPLICATE_ACTION_ID check.
//
// **GENERIC entry point**: handles all instant action types —
// predefined (cancelOrder, startPause, stopPause, stateRequest,
// factsheetRequest, logReport, initPosition, startCharging,
// stopCharging) and custom (any actionType the AGV factsheet
// declares). FMS clients build the desired Action(s) into an
// InstantActions message and send via this service.
//
// Service name: /<topic_namespace>/assign_instant_actions
//   Default <topic_namespace> = "vda5050_master".
//
// **`ASSIGNED` semantics**: pre-flight passed; actions queued for
// publish on the master's queue-processor thread. NOT AGV-acknowledged.
// FMS observes per-action progress via the OrderStatus topic's
// `action_states[]` field (action_status transitions WAITING →
// RUNNING → FINISHED / FAILED).
//
// **Conversion**: `vda5050_interfaces::msg::InstantActions` →
// `vda5050_core::types::InstantActions` via `internal::from_msg<>` (JSON
// intermediate; ENABLE_ROS2 build flag).
//
// **InstantActionsSender callable**: takes `std::function` (not raw
// `Master*`) to keep the service unit-testable without a full Master
// instance. VDA5050MasterROS2 wraps `master.assign_instant_actions`
// via a lambda.

class InstantActionsSendService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "assign_instant_actions";

  /// InstantActions-dispatch callable. Typically wraps
  /// VDA5050Master::assign_instant_actions(...).
  using InstantActionsSender =
    std::function<vda5050_core::master::InstantActionAssignmentResult(
      const std::string& manufacturer, const std::string& serial_number,
      const vda5050_core::types::InstantActions& actions)>;

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the service. Must outlive
  ///                        this object.
  /// \param sender          InstantActions-dispatch callable. Typically
  ///                        wraps VDA5050Master::assign_instant_actions(...).
  /// \param topic_namespace Prefix for the service name. Defaults to
  ///                        "vda5050_master".
  InstantActionsSendService(
    rclcpp::Node::SharedPtr node, InstantActionsSender sender,
    const std::string& topic_namespace = "vda5050_master");

  ~InstantActionsSendService() = default;
  InstantActionsSendService(const InstantActionsSendService&) = delete;
  InstantActionsSendService& operator=(const InstantActionsSendService&) =
    delete;
  InstantActionsSendService(InstantActionsSendService&&) = delete;
  InstantActionsSendService& operator=(InstantActionsSendService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using AssignInstantActions = vda5050_master_ros2::srv::AssignInstantActions;

  void handle_request(
    const std::shared_ptr<AssignInstantActions::Request> request,
    std::shared_ptr<AssignInstantActions::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  InstantActionsSender sender_;
  std::string service_name_;
  rclcpp::Service<AssignInstantActions>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__INSTANT_ACTIONS_SEND_SERVICE_HPP_
