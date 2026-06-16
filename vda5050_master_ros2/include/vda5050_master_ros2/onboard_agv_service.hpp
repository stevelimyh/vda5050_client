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

#ifndef VDA5050_MASTER_ROS2__ONBOARD_AGV_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__ONBOARD_AGV_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_master_ros2/srv/onboard_agv.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OnboardAGVService — single-AGV ROS 2 service for AGV onboarding.
// =============================================================================
//
// Service name: /<topic_namespace>/onboard_agv
//
// Request carries one AGVOnboardSpec. Response is a single status enum
// (SUCCESS / ALREADY_ONBOARDED / INVALID_REQUEST). For onboarding N AGVs
// in one call see OnboardAGVBatchService (the batch sibling). Both services
// route through master.onboard_agv_batch() under the hood.

class OnboardAGVService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "onboard_agv";

  /// Batch onboard dispatcher. Typically wraps
  /// VDA5050Master::onboard_agv_batch — the single-AGV service wraps
  /// the request into a one-element vector internally.
  using OnboardBatcher =
    std::function<vda5050_core::master::VDA5050Master::BatchOnboardResult(
      const std::vector<vda5050_core::master::VDA5050Master::OnboardSpec>&)>;

  OnboardAGVService(
    rclcpp::Node::SharedPtr node, OnboardBatcher batcher,
    const std::string& topic_namespace = "vda5050_master");

  ~OnboardAGVService() = default;
  OnboardAGVService(const OnboardAGVService&) = delete;
  OnboardAGVService& operator=(const OnboardAGVService&) = delete;
  OnboardAGVService(OnboardAGVService&&) = delete;
  OnboardAGVService& operator=(OnboardAGVService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using OnboardAGV = vda5050_master_ros2::srv::OnboardAGV;

  void handle_request(
    const std::shared_ptr<OnboardAGV::Request> request,
    std::shared_ptr<OnboardAGV::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OnboardBatcher batcher_;
  std::string service_name_;
  rclcpp::Service<OnboardAGV>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ONBOARD_AGV_SERVICE_HPP_
