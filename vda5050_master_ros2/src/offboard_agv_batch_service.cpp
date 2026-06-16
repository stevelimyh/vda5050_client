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

#include "vda5050_master_ros2/offboard_agv_batch_service.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {

std::string OffboardAGVBatchService::make_service_name(
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

OffboardAGVBatchService::OffboardAGVBatchService(
  rclcpp::Node::SharedPtr node, OffboardBatcher batcher,
  const std::string& topic_namespace)
: node_(std::move(node)),
  batcher_(std::move(batcher)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<OffboardAGVBatch>(
    service_name_, [this](
                     const std::shared_ptr<OffboardAGVBatch::Request> request,
                     std::shared_ptr<OffboardAGVBatch::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO(
    "[OffboardAGVBatchService] advertised service on {}", service_name_);
}

void OffboardAGVBatchService::handle_request(
  const std::shared_ptr<OffboardAGVBatch::Request> request,
  std::shared_ptr<OffboardAGVBatch::Response> response)
{
  std::vector<std::pair<std::string, std::string>> keys;
  keys.reserve(request->agvs.size());
  for (const auto& k : request->agvs)
  {
    keys.emplace_back(k.manufacturer, k.serial_number);
  }
  const std::size_t removed = batcher_(keys);
  response->offboarded_count = static_cast<std::uint32_t>(removed);
}

}  // namespace vda5050_master_ros2
