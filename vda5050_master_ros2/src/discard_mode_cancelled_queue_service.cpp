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

#include "vda5050_master_ros2/discard_mode_cancelled_queue_service.hpp"

#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {
std::string DiscardModeCancelledQueueService::make_service_name(
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

DiscardModeCancelledQueueService::DiscardModeCancelledQueueService(
  rclcpp::Node::SharedPtr node, DiscardHandler handler,
  const std::string& topic_namespace)
: node_(std::move(node)),
  handler_(std::move(handler)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<DiscardModeCancelledQueue>(
    service_name_,
    [this](
      const std::shared_ptr<DiscardModeCancelledQueue::Request> request,
      std::shared_ptr<DiscardModeCancelledQueue::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO(
    "[DiscardModeCancelledQueueService] advertised service on {}",
    service_name_);
}

void DiscardModeCancelledQueueService::handle_request(
  const std::shared_ptr<DiscardModeCancelledQueue::Request> request,
  std::shared_ptr<DiscardModeCancelledQueue::Response> response)
{
  response->manufacturer = request->manufacturer;
  response->serial_number = request->serial_number;
  response->orders_discarded = 0;
  response->actions_discarded = 0;

  if (request->manufacturer.empty() || request->serial_number.empty())
  {
    response->status = DiscardModeCancelledQueue::Response::INVALID_REQUEST;
    return;
  }

  auto outcome = handler_(request->manufacturer, request->serial_number);
  if (!outcome.has_value())
  {
    response->status = DiscardModeCancelledQueue::Response::AGV_NOT_ONBOARDED;
    return;
  }

  response->status = DiscardModeCancelledQueue::Response::SUCCESS;
  response->orders_discarded = static_cast<uint32_t>(outcome->first);
  response->actions_discarded = static_cast<uint32_t>(outcome->second);
}

}  // namespace vda5050_master_ros2
