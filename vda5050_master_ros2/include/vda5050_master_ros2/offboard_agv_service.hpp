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

#ifndef VDA5050_MASTER_ROS2__OFFBOARD_AGV_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__OFFBOARD_AGV_SERVICE_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/srv/offboard_agv.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OffboardAGVService — single-AGV ROS 2 service for AGV offboarding.
// =============================================================================
//
// Service name: /<topic_namespace>/offboard_agv
//
// Request carries one AGVKey. Response is a single status enum
// (SUCCESS / NOT_ONBOARDED / INVALID_REQUEST). For offboarding N AGVs
// in one call see OffboardAGVBatchService (the batch sibling). Both
// services route through master.offboard_agv_batch() under the hood.

class OffboardAGVService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "offboard_agv";

  /// Batch offboard dispatcher. Typically wraps
  /// VDA5050Master::offboard_agv_batch — the single-AGV service wraps
  /// the request into a one-element vector internally. Returns the
  /// number actually offboarded (keys present in agvs_).
  using OffboardBatcher = std::function<std::size_t(
    const std::vector<std::pair<std::string, std::string>>&)>;

  OffboardAGVService(
    rclcpp::Node::SharedPtr node, OffboardBatcher batcher,
    const std::string& topic_namespace = "vda5050_master");

  ~OffboardAGVService() = default;
  OffboardAGVService(const OffboardAGVService&) = delete;
  OffboardAGVService& operator=(const OffboardAGVService&) = delete;
  OffboardAGVService(OffboardAGVService&&) = delete;
  OffboardAGVService& operator=(OffboardAGVService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using OffboardAGV = vda5050_master_ros2::srv::OffboardAGV;

  void handle_request(
    const std::shared_ptr<OffboardAGV::Request> request,
    std::shared_ptr<OffboardAGV::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OffboardBatcher batcher_;
  std::string service_name_;
  rclcpp::Service<OffboardAGV>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__OFFBOARD_AGV_SERVICE_HPP_
