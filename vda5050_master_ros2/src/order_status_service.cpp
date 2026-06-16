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

#include "vda5050_master_ros2/order_status_service.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master_ros2/order_status_builder.hpp"

namespace vda5050_master_ros2 {
namespace {

builtin_interfaces::msg::Time to_ros_time(
  const vda5050_core::master::AGV::TimePoint& tp)
{
  const auto since_epoch = tp.time_since_epoch();
  const auto secs =
    std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nsecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(secs.count());
  out.nanosec = static_cast<uint32_t>(nsecs.count());
  return out;
}

}  // namespace

std::string OrderStatusService::make_service_name(
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

OrderStatusService::OrderStatusService(
  rclcpp::Node::SharedPtr node, AgvLookup agv_lookup,
  AssignmentLookup assignment_lookup, const std::string& topic_namespace)
: node_(std::move(node)),
  agv_lookup_(std::move(agv_lookup)),
  assignment_lookup_(std::move(assignment_lookup)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<GetOrderStatus>(
    service_name_, [this](
                     const std::shared_ptr<GetOrderStatus::Request> request,
                     std::shared_ptr<GetOrderStatus::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[OrderStatusService] advertised service on {}", service_name_);
}

void OrderStatusService::handle_request(
  const std::shared_ptr<GetOrderStatus::Request> request,
  std::shared_ptr<GetOrderStatus::Response> response)
{
  // Always echo the request key for async-batching correlation.
  response->manufacturer = request->manufacturer;
  response->serial_number = request->serial_number;

  if (request->manufacturer.empty() || request->serial_number.empty())
  {
    response->status = GetOrderStatus::Response::INVALID_REQUEST;
    return;
  }

  std::shared_ptr<vda5050_core::master::AGV> agv =
    agv_lookup_(request->manufacturer, request->serial_number);
  if (!agv)
  {
    response->status = GetOrderStatus::Response::AGV_NOT_ONBOARDED;
    return;
  }

  vda5050_core::master::AGV::OrderStatusBundle bundle =
    agv->get_order_status_bundle();

  if (!bundle.state.has_value() && !bundle.active_order_snapshot.has_active)
  {
    response->status = GetOrderStatus::Response::AGV_SILENT;
    return;
  }

  response->status = GetOrderStatus::Response::SUCCESS;
  const std::string assignment_id =
    assignment_lookup_
      ? assignment_lookup_(request->manufacturer, request->serial_number)
      : std::string{};
  response->order_status = build_order_status_msg(
    bundle, request->manufacturer, request->serial_number, assignment_id);
  if (bundle.state_received_at.has_value())
  {
    response->state_received_at.push_back(
      to_ros_time(*bundle.state_received_at));
  }
}

}  // namespace vda5050_master_ros2
