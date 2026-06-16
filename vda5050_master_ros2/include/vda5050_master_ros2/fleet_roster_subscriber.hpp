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

#ifndef VDA5050_MASTER_ROS2__FLEET_ROSTER_SUBSCRIBER_HPP_
#define VDA5050_MASTER_ROS2__FLEET_ROSTER_SUBSCRIBER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_master_ros2/msg/fleet_roster.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// FleetRosterSubscriber — Device Manager integration.
// =============================================================================
//
// Subscribes to /<topic_namespace>/fleet_roster (RELIABLE +
// TRANSIENT_LOCAL — DM publishes with matching QoS so a fresh master
// receives the latest roster on subscribe). On each message the
// subscriber diffs the desired set against the master's current
// onboarded set and dispatches batch onboard/offboard to converge.

class FleetRosterSubscriber
{
public:
  static constexpr const char* kTopicLeaf = "fleet_roster";
  static constexpr const char* kDefaultNamespace = "vda5050_master";

  /// Snapshot of currently-onboarded AGVs. Typically wraps
  /// VDA5050Master::get_onboarded_agvs.
  using OnboardedSnapshot =
    std::function<std::vector<std::pair<std::string, std::string>>()>;

  /// Batch onboard dispatcher. Typically wraps
  /// VDA5050Master::onboard_agv_batch.
  using OnboardBatcher =
    std::function<vda5050_core::master::VDA5050Master::BatchOnboardResult(
      const std::vector<vda5050_core::master::VDA5050Master::OnboardSpec>&)>;

  /// Batch offboard dispatcher. Typically wraps
  /// VDA5050Master::offboard_agv_batch. Returns the number actually
  /// offboarded.
  using OffboardBatcher = std::function<std::size_t(
    const std::vector<std::pair<std::string, std::string>>&)>;

  FleetRosterSubscriber(
    rclcpp::Node::SharedPtr node, OnboardedSnapshot onboarded_snapshot,
    OnboardBatcher onboard_batcher, OffboardBatcher offboard_batcher,
    const std::string& topic_namespace = kDefaultNamespace);

  ~FleetRosterSubscriber() = default;
  FleetRosterSubscriber(const FleetRosterSubscriber&) = delete;
  FleetRosterSubscriber& operator=(const FleetRosterSubscriber&) = delete;
  FleetRosterSubscriber(FleetRosterSubscriber&&) = delete;
  FleetRosterSubscriber& operator=(FleetRosterSubscriber&&) = delete;

  const std::string& topic_name() const
  {
    return topic_name_;
  }

  /// Most recent roster_id the subscriber has applied. 0 until first
  /// roster arrives.
  std::uint64_t last_applied_roster_id() const;

  /// Echo of the most recent publisher_id. Empty until first roster.
  std::string last_publisher_id() const;

private:
  void on_roster(vda5050_master_ros2::msg::FleetRoster::SharedPtr msg);

  static std::string make_topic_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OnboardedSnapshot onboarded_snapshot_;
  OnboardBatcher onboard_batcher_;
  OffboardBatcher offboard_batcher_;
  std::string topic_name_;
  rclcpp::Subscription<vda5050_master_ros2::msg::FleetRoster>::SharedPtr sub_;

  mutable std::mutex state_mu_;
  std::uint64_t last_applied_roster_id_ = 0;
  std::string last_publisher_id_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__FLEET_ROSTER_SUBSCRIBER_HPP_
