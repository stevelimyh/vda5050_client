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

#ifndef VDA5050_MASTER_ROS2__ONBOARD_AGV_BATCH_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__ONBOARD_AGV_BATCH_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_master_ros2/srv/onboard_agv_batch.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OnboardAGVBatchService — batch ROS 2 service for AGV onboarding.
// =============================================================================
//
// Service name: /<topic_namespace>/onboard_agvs
//
// Request carries an array of AGVOnboardSpec; each entry is processed
// independently (partial success). Wraps
// VDA5050Master::onboard_agv_batch — same dispatcher path used by
// OnboardAGVService (single) and FleetRosterSubscriber.

class OnboardAGVBatchService
{
public:
  static constexpr const char* kServiceLeaf = "onboard_agv_batch";

  using OnboardBatcher =
    std::function<vda5050_core::master::VDA5050Master::BatchOnboardResult(
      const std::vector<vda5050_core::master::VDA5050Master::OnboardSpec>&)>;

  OnboardAGVBatchService(
    rclcpp::Node::SharedPtr node, OnboardBatcher batcher,
    const std::string& topic_namespace = "vda5050_master");

  ~OnboardAGVBatchService() = default;
  OnboardAGVBatchService(const OnboardAGVBatchService&) = delete;
  OnboardAGVBatchService& operator=(const OnboardAGVBatchService&) = delete;
  OnboardAGVBatchService(OnboardAGVBatchService&&) = delete;
  OnboardAGVBatchService& operator=(OnboardAGVBatchService&&) = delete;

  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using OnboardAGVBatch = vda5050_master_ros2::srv::OnboardAGVBatch;

  void handle_request(
    const std::shared_ptr<OnboardAGVBatch::Request> request,
    std::shared_ptr<OnboardAGVBatch::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OnboardBatcher batcher_;
  std::string service_name_;
  rclcpp::Service<OnboardAGVBatch>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ONBOARD_AGV_BATCH_SERVICE_HPP_
