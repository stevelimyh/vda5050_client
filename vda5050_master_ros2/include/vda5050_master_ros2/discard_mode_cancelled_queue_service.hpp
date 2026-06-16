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

#ifndef VDA5050_MASTER_ROS2__DISCARD_MODE_CANCELLED_QUEUE_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__DISCARD_MODE_CANCELLED_QUEUE_SERVICE_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/srv/discard_mode_cancelled_queue.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// DiscardModeCancelledQueueService (Task #24 — operability surface).
// =============================================================================
//
// Synchronous ROS 2 service that drops the AGV's mode-cancelled
// buffer without re-enqueue. Wraps `AGV::discard_mode_cancelled_queue()`.
//
// Service name: /<topic_namespace>/discard_mode_cancelled_queue
//   Default <topic_namespace> = "vda5050_master".
//
// Counterpart to ResumeModeCancelledQueueService. Both share the
// same handler shape — nullopt result indicates AGV not onboarded.

class DiscardModeCancelledQueueService
{
public:
  static constexpr const char* kServiceLeaf = "discard_mode_cancelled_queue";

  using DiscardHandler =
    std::function<std::optional<std::pair<std::size_t, std::size_t>>(
      const std::string& manufacturer, const std::string& serial_number)>;

  DiscardModeCancelledQueueService(
    rclcpp::Node::SharedPtr node, DiscardHandler handler,
    const std::string& topic_namespace = "vda5050_master");

  ~DiscardModeCancelledQueueService() = default;
  DiscardModeCancelledQueueService(const DiscardModeCancelledQueueService&) =
    delete;
  DiscardModeCancelledQueueService& operator=(
    const DiscardModeCancelledQueueService&) = delete;
  DiscardModeCancelledQueueService(DiscardModeCancelledQueueService&&) = delete;
  DiscardModeCancelledQueueService& operator=(
    DiscardModeCancelledQueueService&&) = delete;

  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using DiscardModeCancelledQueue =
    vda5050_master_ros2::srv::DiscardModeCancelledQueue;

  void handle_request(
    const std::shared_ptr<DiscardModeCancelledQueue::Request> request,
    std::shared_ptr<DiscardModeCancelledQueue::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  DiscardHandler handler_;
  std::string service_name_;
  rclcpp::Service<DiscardModeCancelledQueue>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__DISCARD_MODE_CANCELLED_QUEUE_SERVICE_HPP_
