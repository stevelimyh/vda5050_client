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

#ifndef VDA5050_MASTER_ROS2__OFFBOARD_AGV_BATCH_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__OFFBOARD_AGV_BATCH_SERVICE_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/srv/offboard_agv_batch.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OffboardAGVBatchService — batch ROS 2 service for AGV offboarding.
// =============================================================================
//
// Service name: /<topic_namespace>/offboard_agvs
//
// Request carries an array of AGVKey. Each present key is offboarded;
// missing or empty-key entries are silently ignored (idempotent).
// Wraps VDA5050Master::offboard_agv_batch — same path used by
// OffboardAGVService (single) and FleetRosterSubscriber.

class OffboardAGVBatchService
{
public:
  static constexpr const char* kServiceLeaf = "offboard_agv_batch";

  using OffboardBatcher = std::function<std::size_t(
    const std::vector<std::pair<std::string, std::string>>&)>;

  OffboardAGVBatchService(
    rclcpp::Node::SharedPtr node, OffboardBatcher batcher,
    const std::string& topic_namespace = "vda5050_master");

  ~OffboardAGVBatchService() = default;
  OffboardAGVBatchService(const OffboardAGVBatchService&) = delete;
  OffboardAGVBatchService& operator=(const OffboardAGVBatchService&) = delete;
  OffboardAGVBatchService(OffboardAGVBatchService&&) = delete;
  OffboardAGVBatchService& operator=(OffboardAGVBatchService&&) = delete;

  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using OffboardAGVBatch = vda5050_master_ros2::srv::OffboardAGVBatch;

  void handle_request(
    const std::shared_ptr<OffboardAGVBatch::Request> request,
    std::shared_ptr<OffboardAGVBatch::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OffboardBatcher batcher_;
  std::string service_name_;
  rclcpp::Service<OffboardAGVBatch>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__OFFBOARD_AGV_BATCH_SERVICE_HPP_
