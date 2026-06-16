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

#include "vda5050_master_ros2/onboard_agv_batch_service.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master_ros2/msg/agv_onboard_spec.hpp"

namespace vda5050_master_ros2 {
namespace {

vda5050_master_ros2::msg::AGVOnboardSpec spec_to_entry(
  const vda5050_core::master::VDA5050Master::OnboardSpec& spec)
{
  vda5050_master_ros2::msg::AGVOnboardSpec e;
  e.manufacturer = spec.manufacturer;
  e.serial_number = spec.serial_number;
  e.max_queue_size = static_cast<std::uint32_t>(spec.max_queue_size);
  e.drop_oldest = spec.drop_oldest;
  return e;
}

}  // namespace

std::string OnboardAGVBatchService::make_service_name(
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

OnboardAGVBatchService::OnboardAGVBatchService(
  rclcpp::Node::SharedPtr node, OnboardBatcher batcher,
  const std::string& topic_namespace)
: node_(std::move(node)),
  batcher_(std::move(batcher)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<OnboardAGVBatch>(
    service_name_, [this](
                     const std::shared_ptr<OnboardAGVBatch::Request> request,
                     std::shared_ptr<OnboardAGVBatch::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO(
    "[OnboardAGVBatchService] advertised service on {}", service_name_);
}

void OnboardAGVBatchService::handle_request(
  const std::shared_ptr<OnboardAGVBatch::Request> request,
  std::shared_ptr<OnboardAGVBatch::Response> response)
{
  using vda5050_core::master::VDA5050Master;

  std::vector<VDA5050Master::OnboardSpec> specs;
  specs.reserve(request->agvs.size());
  for (const auto& entry : request->agvs)
  {
    VDA5050Master::OnboardSpec spec;
    spec.manufacturer = entry.manufacturer;
    spec.serial_number = entry.serial_number;
    // Wire sentinel 0 → keep the master-side default (10).
    if (entry.max_queue_size != 0)
    {
      spec.max_queue_size = entry.max_queue_size;
    }
    spec.drop_oldest = entry.drop_oldest;
    specs.push_back(spec);
  }

  auto result = batcher_(specs);

  response->onboarded.reserve(result.onboarded.size());
  for (const auto& s : result.onboarded)
  {
    response->onboarded.push_back(spec_to_entry(s));
  }
  response->failed.reserve(result.failed.size());
  for (const auto& s : result.failed)
  {
    response->failed.push_back(spec_to_entry(s));
  }
}

}  // namespace vda5050_master_ros2
