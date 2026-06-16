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

#include "vda5050_master_ros2/onboard_agv_service.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {

std::string OnboardAGVService::make_service_name(
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

OnboardAGVService::OnboardAGVService(
  rclcpp::Node::SharedPtr node, OnboardBatcher batcher,
  const std::string& topic_namespace)
: node_(std::move(node)),
  batcher_(std::move(batcher)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<OnboardAGV>(
    service_name_, [this](
                     const std::shared_ptr<OnboardAGV::Request> request,
                     std::shared_ptr<OnboardAGV::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[OnboardAGVService] advertised service on {}", service_name_);
}

void OnboardAGVService::handle_request(
  const std::shared_ptr<OnboardAGV::Request> request,
  std::shared_ptr<OnboardAGV::Response> response)
{
  using vda5050_core::master::VDA5050Master;

  VDA5050_INFO(
    "[OnboardAGVService] ENTRY mfg={} sn={}", request->agv.manufacturer,
    request->agv.serial_number);

  response->manufacturer = request->agv.manufacturer;
  response->serial_number = request->agv.serial_number;

  VDA5050Master::OnboardSpec spec;
  spec.manufacturer = request->agv.manufacturer;
  spec.serial_number = request->agv.serial_number;
  // Wire sentinel 0 → keep the master-side default (10).
  if (request->agv.max_queue_size != 0)
  {
    spec.max_queue_size = request->agv.max_queue_size;
  }
  spec.drop_oldest = request->agv.drop_oldest;

  VDA5050_INFO("[OnboardAGVService] calling batcher_");
  auto result = batcher_({spec});
  VDA5050_INFO(
    "[OnboardAGVService] batcher_ returned: onboarded={} skipped={} failed={}",
    result.onboarded.size(), result.skipped_already_onboarded.size(),
    result.failed.size());

  if (!result.failed.empty())
  {
    response->status = OnboardAGV::Response::INVALID_REQUEST;
  }
  else if (!result.skipped_already_onboarded.empty())
  {
    response->status = OnboardAGV::Response::ALREADY_ONBOARDED;
  }
  else
  {
    response->status = OnboardAGV::Response::SUCCESS;
  }

  VDA5050_INFO(
    "[OnboardAGVService] EXIT status={}", static_cast<int>(response->status));
}

}  // namespace vda5050_master_ros2
