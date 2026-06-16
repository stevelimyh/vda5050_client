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

#include "vda5050_master_ros2/offboard_agv_service.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {

std::string OffboardAGVService::make_service_name(
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

OffboardAGVService::OffboardAGVService(
  rclcpp::Node::SharedPtr node, OffboardBatcher batcher,
  const std::string& topic_namespace)
: node_(std::move(node)),
  batcher_(std::move(batcher)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<OffboardAGV>(
    service_name_, [this](
                     const std::shared_ptr<OffboardAGV::Request> request,
                     std::shared_ptr<OffboardAGV::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[OffboardAGVService] advertised service on {}", service_name_);
}

void OffboardAGVService::handle_request(
  const std::shared_ptr<OffboardAGV::Request> request,
  std::shared_ptr<OffboardAGV::Response> response)
{
  response->manufacturer = request->agv.manufacturer;
  response->serial_number = request->agv.serial_number;

  if (request->agv.manufacturer.empty() || request->agv.serial_number.empty())
  {
    response->status = OffboardAGV::Response::INVALID_REQUEST;
    return;
  }

  std::vector<std::pair<std::string, std::string>> keys = {
    {request->agv.manufacturer, request->agv.serial_number}};
  const std::size_t removed = batcher_(keys);
  response->status = (removed == 1) ? OffboardAGV::Response::SUCCESS
                                    : OffboardAGV::Response::NOT_ONBOARDED;
}

}  // namespace vda5050_master_ros2
