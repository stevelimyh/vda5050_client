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

#ifndef VDA5050_MASTER_ROS2__RESUME_MODE_CANCELLED_QUEUE_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__RESUME_MODE_CANCELLED_QUEUE_SERVICE_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/srv/resume_mode_cancelled_queue.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// ResumeModeCancelledQueueService (Task #24 — operability surface).
// =============================================================================
//
// Synchronous ROS 2 service that prepends the captured mode-cancelled
// queue back to the live queue (preserves original FMS-intended
// ordering). Wraps `AGV::resume_mode_cancelled_queue()`.
//
// Service name: /<topic_namespace>/resume_mode_cancelled_queue
//   Default <topic_namespace> = "vda5050_master".
//
// Pure-ROS-2 FMS deployments call this on the AUTOMATIC return edge
// to recover the queue contents the library captured at the prior
// leave-AUTOMATIC. C++ FMS subclasses can call
// `agv->resume_mode_cancelled_queue()` directly instead.
//
// **ResumeHandler callable**: takes `std::function<{count,count}(mfg,
// serial)>` for unit-testability — no Master instance required for
// tests. VDA5050MasterROS2 wraps `get_agv` + `resume_mode_cancelled_queue`
// via a single lambda. Returning {0, 0} for a missing AGV is
// distinguished from a real "buffer empty" success via the
// optional<> wrapper: nullopt = AGV not onboarded.

class ResumeModeCancelledQueueService
{
public:
  static constexpr const char* kServiceLeaf = "resume_mode_cancelled_queue";

  /// Resume handler callable. Returns nullopt if no AGV with this
  /// {mfg, serial} is onboarded; otherwise returns the
  /// {orders_resumed, actions_resumed} pair from
  /// AGV::resume_mode_cancelled_queue.
  using ResumeHandler =
    std::function<std::optional<std::pair<std::size_t, std::size_t>>(
      const std::string& manufacturer, const std::string& serial_number)>;

  ResumeModeCancelledQueueService(
    rclcpp::Node::SharedPtr node, ResumeHandler handler,
    const std::string& topic_namespace = "vda5050_master");

  ~ResumeModeCancelledQueueService() = default;
  ResumeModeCancelledQueueService(const ResumeModeCancelledQueueService&) =
    delete;
  ResumeModeCancelledQueueService& operator=(
    const ResumeModeCancelledQueueService&) = delete;
  ResumeModeCancelledQueueService(ResumeModeCancelledQueueService&&) = delete;
  ResumeModeCancelledQueueService& operator=(
    ResumeModeCancelledQueueService&&) = delete;

  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using ResumeModeCancelledQueue =
    vda5050_master_ros2::srv::ResumeModeCancelledQueue;

  void handle_request(
    const std::shared_ptr<ResumeModeCancelledQueue::Request> request,
    std::shared_ptr<ResumeModeCancelledQueue::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  ResumeHandler handler_;
  std::string service_name_;
  rclcpp::Service<ResumeModeCancelledQueue>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__RESUME_MODE_CANCELLED_QUEUE_SERVICE_HPP_
