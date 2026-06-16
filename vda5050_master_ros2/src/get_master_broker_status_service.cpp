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

#include "vda5050_master_ros2/get_master_broker_status_service.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {
namespace {

builtin_interfaces::msg::Time to_ros_time(
  const std::chrono::system_clock::time_point& tp)
{
  const auto since_epoch = tp.time_since_epoch();
  const auto secs =
    std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nsecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<std::int32_t>(secs.count());
  out.nanosec = static_cast<std::uint32_t>(nsecs.count());
  return out;
}

}  // namespace

std::string GetMasterBrokerStatusService::make_service_name(
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

GetMasterBrokerStatusService::GetMasterBrokerStatusService(
  rclcpp::Node::SharedPtr node, StatusLookup status_lookup,
  const std::string& topic_namespace)
: node_(std::move(node)),
  status_lookup_(std::move(status_lookup)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<GetMasterBrokerStatus>(
    service_name_,
    [this](
      const std::shared_ptr<GetMasterBrokerStatus::Request> request,
      std::shared_ptr<GetMasterBrokerStatus::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO(
    "[GetMasterBrokerStatusService] advertised service on {}", service_name_);
}

void GetMasterBrokerStatusService::handle_request(
  const std::shared_ptr<GetMasterBrokerStatus::Request> /*request*/,
  std::shared_ptr<GetMasterBrokerStatus::Response> response)
{
  const StatusSnapshot snap =
    status_lookup_ ? status_lookup_() : StatusSnapshot{};

  response->connected = snap.connected;
  response->reconnect_count = snap.reconnect_count;
  if (snap.last_disconnect_at.has_value())
  {
    response->last_disconnect_at.push_back(
      to_ros_time(*snap.last_disconnect_at));
  }
}

}  // namespace vda5050_master_ros2
