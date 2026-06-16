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

#include "vda5050_master_ros2/instant_actions_send_service.hpp"

#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_interfaces/msg/error.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"

namespace vda5050_master_ros2 {
namespace {

// 1:1 mapping from C++ InstantActionDecision to the .srv response
// decision enum. No `default:` — the compiler enforces exhaustiveness
// via -Wswitch.
uint8_t map_decision(vda5050_core::master::InstantActionDecision d)
{
  using Resp = vda5050_master_ros2::srv::AssignInstantActions::Response;
  switch (d)
  {
    case vda5050_core::master::InstantActionDecision::ASSIGNED:
      return Resp::ASSIGNED;
    case vda5050_core::master::InstantActionDecision::AGV_NOT_ONBOARDED:
      return Resp::AGV_NOT_ONBOARDED;
    case vda5050_core::master::InstantActionDecision::AGV_OFFLINE:
      return Resp::AGV_OFFLINE;
    case vda5050_core::master::InstantActionDecision::DUPLICATE_ACTION_ID:
      return Resp::DUPLICATE_ACTION_ID;
    case vda5050_core::master::InstantActionDecision::AGV_QUEUE_FULL:
      return Resp::AGV_QUEUE_FULL;
    case vda5050_core::master::InstantActionDecision::HARD_ACTION_BLOCKED:
      return Resp::HARD_ACTION_BLOCKED;
    case vda5050_core::master::InstantActionDecision::ACTION_BLOCKED_BY_DRIVING:
      return Resp::ACTION_BLOCKED_BY_DRIVING;
    case vda5050_core::master::InstantActionDecision::
      AGV_MODE_NOT_AUTO_FOR_ACTION:
      return Resp::AGV_MODE_NOT_AUTO_FOR_ACTION;
  }
  // Defensive — unreachable given enum exhaustiveness above. If we
  // reach here, the C++ enum gained a value without updating this
  // switch; treat as INVALID_REQUEST so the FMS sees something.
  return Resp::INVALID_REQUEST;
}

}  // namespace

std::string InstantActionsSendService::make_service_name(
  const std::string& topic_namespace)
{
  std::string name = "/";
  if (!topic_namespace.empty())
  {
    name += topic_namespace;
    name += "/";
  }
  name += kServiceLeaf;
  return name;
}

InstantActionsSendService::InstantActionsSendService(
  rclcpp::Node::SharedPtr node, InstantActionsSender sender,
  const std::string& topic_namespace)
: node_(std::move(node)),
  sender_(std::move(sender)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<AssignInstantActions>(
    service_name_,
    [this](
      const std::shared_ptr<AssignInstantActions::Request> request,
      std::shared_ptr<AssignInstantActions::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO(
    "[InstantActionsSendService] advertised service on {}", service_name_);
}

void InstantActionsSendService::handle_request(
  const std::shared_ptr<AssignInstantActions::Request> request,
  std::shared_ptr<AssignInstantActions::Response> response)
{
  // Echo identity for FMS-side correlation.
  response->manufacturer = request->manufacturer;
  response->serial_number = request->serial_number;

  if (request->manufacturer.empty() || request->serial_number.empty())
  {
    response->decision = AssignInstantActions::Response::INVALID_REQUEST;
    return;
  }

  // Convert msg::InstantActions → types::InstantActions via the
  // JSON-intermediate template (ENABLE_ROS2 build flag).
  vda5050_core::types::InstantActions actions_typed = internal::from_msg<
    vda5050_interfaces::msg::InstantActions,
    vda5050_core::types::InstantActions>(request->instant_actions);

  vda5050_core::master::InstantActionAssignmentResult result =
    sender_(request->manufacturer, request->serial_number, actions_typed);

  response->decision = map_decision(result.decision);

  response->errors.reserve(result.errors.size());
  for (const auto& err : result.errors)
  {
    response->errors.push_back(
      internal::to_msg<
        vda5050_core::types::Error, vda5050_interfaces::msg::Error>(err));
  }
}

}  // namespace vda5050_master_ros2
