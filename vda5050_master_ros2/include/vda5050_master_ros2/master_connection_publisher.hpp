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

#ifndef VDA5050_MASTER_ROS2__MASTER_CONNECTION_PUBLISHER_HPP_
#define VDA5050_MASTER_ROS2__MASTER_CONNECTION_PUBLISHER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/msg/master_connection.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// MasterConnectionPublisher — DM-side liveness signal.
// =============================================================================
//
// Publishes /<topic_namespace>/master_connection (RELIABLE +
// TRANSIENT_LOCAL). State transitions:
//
//   STARTING (on construction)
//     → READY (caller sets after broker connects + master is ready)
//     → DEGRADED (caller sets on broker disconnect)
//     → READY (caller sets on broker reconnect)
//     → SHUTTING_DOWN (destructor — best-effort)
//
// A 30s heartbeat timer republishes the current state so DM can
// distinguish "master crashed" from "master is quiet because nothing
// changed".

class MasterConnectionPublisher
{
public:
  static constexpr const char* kTopicLeaf = "master_connection";
  static constexpr const char* kDefaultNamespace = "vda5050_master";
  static constexpr std::chrono::seconds kHeartbeatInterval{30};

  /// Live broker reachability lookup. Typically wraps
  /// `VDA5050Master::get_broker_status().connected`.
  using BrokerConnectedLookup = std::function<bool()>;

  /// Live onboarded-AGV count lookup. Typically wraps
  /// `master.get_onboarded_agvs().size()`.
  using OnboardedCountLookup = std::function<std::uint32_t()>;

  MasterConnectionPublisher(
    rclcpp::Node::SharedPtr node, const std::string& master_id,
    const std::string& master_version,
    BrokerConnectedLookup broker_connected_lookup,
    OnboardedCountLookup onboarded_count_lookup,
    const std::string& topic_namespace = kDefaultNamespace);

  ~MasterConnectionPublisher();
  MasterConnectionPublisher(const MasterConnectionPublisher&) = delete;
  MasterConnectionPublisher& operator=(const MasterConnectionPublisher&) =
    delete;
  MasterConnectionPublisher(MasterConnectionPublisher&&) = delete;
  MasterConnectionPublisher& operator=(MasterConnectionPublisher&&) = delete;

  /// Set the current state and publish immediately. No-op if the
  /// state is unchanged. Thread-safe.
  void set_state(std::uint8_t new_state);

  /// Read the current state. Test + diagnostic helper.
  std::uint8_t current_state() const;

  const std::string& topic_name() const
  {
    return topic_name_;
  }

private:
  void publish_current_locked_();
  void heartbeat_tick_();

  static std::string make_topic_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  std::string master_id_;
  std::string master_version_;
  BrokerConnectedLookup broker_connected_lookup_;
  OnboardedCountLookup onboarded_count_lookup_;
  std::string topic_name_;

  mutable std::mutex state_mu_;
  std::uint8_t current_state_;

  rclcpp::Publisher<vda5050_master_ros2::msg::MasterConnection>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__MASTER_CONNECTION_PUBLISHER_HPP_
