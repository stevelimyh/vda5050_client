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

#include "vda5050_master_ros2/device_status_service.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_interfaces/msg/connection.hpp"
#include "vda5050_interfaces/msg/factsheet.hpp"
#include "vda5050_interfaces/msg/state.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"

namespace vda5050_master_ros2 {
namespace {

using AgvClock = vda5050_core::master::AGV::Clock;
using AgvTimePoint = vda5050_core::master::AGV::TimePoint;

builtin_interfaces::msg::Time to_ros_time(const AgvTimePoint& tp)
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

std::string DeviceStatusService::make_service_name(
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

DeviceStatusService::DeviceStatusService(
  rclcpp::Node::SharedPtr node, AgvLookup agv_lookup,
  const std::string& topic_namespace)
: node_(std::move(node)),
  agv_lookup_(std::move(agv_lookup)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<GetDeviceStatus>(
    service_name_, [this](
                     const std::shared_ptr<GetDeviceStatus::Request> request,
                     std::shared_ptr<GetDeviceStatus::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[DeviceStatusService] advertised service on {}", service_name_);
}

void DeviceStatusService::handle_request(
  const std::shared_ptr<GetDeviceStatus::Request> request,
  std::shared_ptr<GetDeviceStatus::Response> response)
{
  // Echo the request key so async batching clients can correlate.
  response->manufacturer = request->manufacturer;
  response->serial_number = request->serial_number;
  response->state.clear();
  response->connection.clear();
  response->factsheet.clear();

  if (request->manufacturer.empty() || request->serial_number.empty())
  {
    response->status = GetDeviceStatus::Response::INVALID_REQUEST;
    return;
  }

  std::shared_ptr<vda5050_core::master::AGV> agv =
    agv_lookup_(request->manufacturer, request->serial_number);
  if (!agv)
  {
    response->status = GetDeviceStatus::Response::AGV_NOT_ONBOARDED;
    return;
  }

  vda5050_core::master::AGV::StatusSnapshot snap = agv->get_status_snapshot();

  if (!snap.state.has_value() && !snap.connection.has_value())
  {
    response->status = GetDeviceStatus::Response::AGV_SILENT;
    return;
  }

  response->status = GetDeviceStatus::Response::SUCCESS;

  if (snap.state.has_value())
  {
    response->state.push_back(
      internal::to_msg<
        vda5050_core::types::State, vda5050_interfaces::msg::State>(
        *snap.state));
    if (snap.state_received_at.has_value())
    {
      response->state_received_at.push_back(
        to_ros_time(*snap.state_received_at));
    }
  }

  if (snap.connection.has_value())
  {
    response->connection.push_back(
      internal::to_msg<
        vda5050_core::types::Connection, vda5050_interfaces::msg::Connection>(
        *snap.connection));
    if (snap.connection_received_at.has_value())
    {
      response->connection_received_at.push_back(
        to_ros_time(*snap.connection_received_at));
    }
  }

  if (snap.factsheet.has_value())
  {
    response->factsheet.push_back(
      internal::to_msg<
        vda5050_core::types::Factsheet, vda5050_interfaces::msg::Factsheet>(
        *snap.factsheet));
    if (snap.factsheet_received_at.has_value())
    {
      response->factsheet_received_at.push_back(
        to_ros_time(*snap.factsheet_received_at));
    }
  }
}

}  // namespace vda5050_master_ros2
