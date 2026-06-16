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

#include "vda5050_master_ros2/order_send_service.hpp"

#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_interfaces/msg/error.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"

namespace vda5050_master_ros2 {
namespace {

// 1:1 mapping from C++ AssignmentDecision to the .srv response decision
// enum. No `default:` — the compiler enforces exhaustiveness via
// -Wswitch, so adding a new enumerator triggers a build error here
// instead of a silent fall-through.
uint8_t map_decision(vda5050_core::master::AssignmentDecision d)
{
  using Resp = vda5050_master_ros2::srv::AssignOrder::Response;
  switch (d)
  {
    case vda5050_core::master::AssignmentDecision::ASSIGNED:
      return Resp::ASSIGNED;
    case vda5050_core::master::AssignmentDecision::AGV_NOT_ONBOARDED:
      return Resp::AGV_NOT_ONBOARDED;
    case vda5050_core::master::AssignmentDecision::AGV_OFFLINE:
      return Resp::AGV_OFFLINE;
    case vda5050_core::master::AssignmentDecision::AGV_NOT_READY:
      return Resp::AGV_NOT_READY;
    case vda5050_core::master::AssignmentDecision::AGV_MODE_NOT_AUTO:
      return Resp::AGV_MODE_NOT_AUTO;
    case vda5050_core::master::AssignmentDecision::AGV_POSITION_NOT_INITIALIZED:
      return Resp::AGV_POSITION_NOT_INITIALIZED;
    case vda5050_core::master::AssignmentDecision::AGV_NO_STATE_YET:
      return Resp::AGV_NO_STATE_YET;
    case vda5050_core::master::AssignmentDecision::STITCH_REJECTED:
      return Resp::STITCH_REJECTED;
    case vda5050_core::master::AssignmentDecision::STITCH_QUEUED:
      return Resp::STITCH_QUEUED;
  }
  // Defensive — unreachable given enum exhaustiveness above. If we
  // reach here, it means the C++ enum gained a value without updating
  // this switch; treat as INVALID_REQUEST so the FMS sees something.
  return Resp::INVALID_REQUEST;
}

}  // namespace

std::string OrderSendService::make_service_name(
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

OrderSendService::OrderSendService(
  rclcpp::Node::SharedPtr node, OrderSender sender,
  const std::string& topic_namespace)
: node_(std::move(node)),
  sender_(std::move(sender)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<AssignOrder>(
    service_name_, [this](
                     const std::shared_ptr<AssignOrder::Request> request,
                     std::shared_ptr<AssignOrder::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[OrderSendService] advertised service on {}", service_name_);
}

void OrderSendService::handle_request(
  const std::shared_ptr<AssignOrder::Request> request,
  std::shared_ptr<AssignOrder::Response> response)
{
  // Echo identity + order id pre-conversion — robust to a malformed
  // order msg that fails JSON conversion (FMS still gets correlation
  // keys back).
  response->manufacturer = request->manufacturer;
  response->serial_number = request->serial_number;
  response->order_id = request->order.order_id;
  response->order_update_id = request->order.order_update_id;

  if (request->manufacturer.empty() || request->serial_number.empty())
  {
    response->decision = AssignOrder::Response::INVALID_REQUEST;
    return;
  }

  // Convert msg::Order → types::Order via the JSON-intermediate
  // template (ENABLE_ROS2 build flag).
  vda5050_core::types::Order order_typed = internal::from_msg<
    vda5050_interfaces::msg::Order, vda5050_core::types::Order>(request->order);

  vda5050_core::master::AssignmentResult result =
    sender_(request->manufacturer, request->serial_number, order_typed);

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
