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

#include "vda5050_master_ros2/fleet_roster_subscriber.hpp"

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {

std::string FleetRosterSubscriber::make_topic_name(
  const std::string& topic_namespace)
{
  std::string name = "/";
  if (!topic_namespace.empty())
  {
    name += topic_namespace;
    name += "/";
  }
  name += kTopicLeaf;
  return name;
}

FleetRosterSubscriber::FleetRosterSubscriber(
  rclcpp::Node::SharedPtr node, OnboardedSnapshot onboarded_snapshot,
  OnboardBatcher onboard_batcher, OffboardBatcher offboard_batcher,
  const std::string& topic_namespace)
: node_(std::move(node)),
  onboarded_snapshot_(std::move(onboarded_snapshot)),
  onboard_batcher_(std::move(onboard_batcher)),
  offboard_batcher_(std::move(offboard_batcher)),
  topic_name_(make_topic_name(topic_namespace))
{
  // RELIABLE + TRANSIENT_LOCAL: DM publishes with the same QoS so a
  // restarted master gets the latched roster on subscribe.
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();

  sub_ = node_->create_subscription<vda5050_master_ros2::msg::FleetRoster>(
    topic_name_, qos,
    [this](vda5050_master_ros2::msg::FleetRoster::SharedPtr msg) {
      this->on_roster(msg);
    });

  VDA5050_INFO("[FleetRosterSubscriber] subscribed to {}", topic_name_);
}

void FleetRosterSubscriber::on_roster(
  vda5050_master_ros2::msg::FleetRoster::SharedPtr msg)
{
  using AGVKey = std::pair<std::string, std::string>;
  using vda5050_core::master::VDA5050Master;

  VDA5050_INFO(
    "[FleetRosterSubscriber] received roster_id={} from {} with {} AGVs",
    msg->roster_id, msg->publisher_id, msg->agvs.size());

  std::set<AGVKey> desired_set;
  std::vector<VDA5050Master::OnboardSpec> add_specs;

  auto current_pairs =
    onboarded_snapshot_ ? onboarded_snapshot_() : std::vector<AGVKey>{};
  std::set<AGVKey> current_set(current_pairs.begin(), current_pairs.end());

  for (const auto& entry : msg->agvs)
  {
    if (entry.manufacturer.empty() || entry.serial_number.empty())
    {
      VDA5050_WARN("[FleetRosterSubscriber] skipping entry with empty mfg/sn");
      continue;
    }
    AGVKey key{entry.manufacturer, entry.serial_number};
    desired_set.insert(key);
    if (current_set.find(key) == current_set.end())
    {
      VDA5050Master::OnboardSpec spec;
      spec.manufacturer = entry.manufacturer;
      spec.serial_number = entry.serial_number;
      // AGVOnboardSpec treats max_queue_size==0 as "use master default".
      if (entry.max_queue_size != 0)
      {
        spec.max_queue_size = entry.max_queue_size;
      }
      spec.drop_oldest = entry.drop_oldest;
      add_specs.push_back(spec);
    }
  }

  std::vector<AGVKey> remove_keys;
  for (const auto& key : current_set)
  {
    if (desired_set.find(key) == desired_set.end())
    {
      remove_keys.push_back(key);
    }
  }

  // Onboard before offboard so new AGVs are live before old ones go
  // away (matters if a roster swaps one identity for another and the
  // caller treats them as distinct).
  if (!add_specs.empty() && onboard_batcher_)
  {
    auto result = onboard_batcher_(add_specs);
    VDA5050_INFO(
      "[FleetRosterSubscriber] batch onboard: {} added, {} skipped, {} failed",
      result.onboarded.size(), result.skipped_already_onboarded.size(),
      result.failed.size());
  }
  if (!remove_keys.empty() && offboard_batcher_)
  {
    const auto removed = offboard_batcher_(remove_keys);
    VDA5050_INFO("[FleetRosterSubscriber] batch offboard: {} removed", removed);
  }

  std::lock_guard<std::mutex> guard(state_mu_);
  last_applied_roster_id_ = msg->roster_id;
  last_publisher_id_ = msg->publisher_id;
}

std::uint64_t FleetRosterSubscriber::last_applied_roster_id() const
{
  std::lock_guard<std::mutex> guard(state_mu_);
  return last_applied_roster_id_;
}

std::string FleetRosterSubscriber::last_publisher_id() const
{
  std::lock_guard<std::mutex> guard(state_mu_);
  return last_publisher_id_;
}

}  // namespace vda5050_master_ros2
