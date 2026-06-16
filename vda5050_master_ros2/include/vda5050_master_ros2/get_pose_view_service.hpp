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

#ifndef VDA5050_MASTER_ROS2__GET_POSE_VIEW_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__GET_POSE_VIEW_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_master_ros2/srv/get_pose_view.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// GetPoseViewService (Task #78 / VM-AMQP-10 — service half).
// =============================================================================
//
// Synchronous ROS 2 service that returns the master's fused pose snapshot for
// one AGV — the same view the PoseViewPublisher stream emits.
//
// Service name: /<topic_namespace>/get_pose_view
//   Default <topic_namespace> = "vda5050_master".
//
// Status enum (in vda5050_master_ros2/srv/GetPoseView):
//   SUCCESS           — pose_view payload populated.
//   INVALID_REQUEST   — empty manufacturer or serial_number.
//   AGV_NOT_ONBOARDED — lookup returned nullptr.
//   AGV_SILENT        — onboarded but no position received yet
//                       (source would be SOURCE_NONE).

class GetPoseViewService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "get_pose_view";

  /// AGV lookup callable. Returns nullptr when no AGV with the given
  /// {manufacturer, serial_number} is currently onboarded.
  using AgvLookup = std::function<std::shared_ptr<vda5050_core::master::AGV>(
    const std::string& manufacturer, const std::string& serial_number)>;

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the service. Must outlive
  ///                        this object.
  /// \param agv_lookup      Lookup callable. Typically wraps
  ///                        VDA5050Master::get_agv(...).
  /// \param topic_namespace Prefix for the service name. Defaults to
  ///                        "vda5050_master".
  GetPoseViewService(
    rclcpp::Node::SharedPtr node, AgvLookup agv_lookup,
    const std::string& topic_namespace = "vda5050_master");

  ~GetPoseViewService() = default;
  GetPoseViewService(const GetPoseViewService&) = delete;
  GetPoseViewService& operator=(const GetPoseViewService&) = delete;
  GetPoseViewService(GetPoseViewService&&) = delete;
  GetPoseViewService& operator=(GetPoseViewService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using GetPoseView = vda5050_master_ros2::srv::GetPoseView;

  void handle_request(
    const std::shared_ptr<GetPoseView::Request> request,
    std::shared_ptr<GetPoseView::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  AgvLookup agv_lookup_;
  std::string service_name_;
  rclcpp::Service<GetPoseView>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__GET_POSE_VIEW_SERVICE_HPP_
