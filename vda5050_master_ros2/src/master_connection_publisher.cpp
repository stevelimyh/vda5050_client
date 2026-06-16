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

#include "vda5050_master_ros2/master_connection_publisher.hpp"

#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {

std::string MasterConnectionPublisher::make_topic_name(
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

MasterConnectionPublisher::MasterConnectionPublisher(
  rclcpp::Node::SharedPtr node, const std::string& master_id,
  const std::string& master_version,
  BrokerConnectedLookup broker_connected_lookup,
  OnboardedCountLookup onboarded_count_lookup,
  const std::string& topic_namespace)
: node_(std::move(node)),
  master_id_(master_id),
  master_version_(master_version),
  broker_connected_lookup_(std::move(broker_connected_lookup)),
  onboarded_count_lookup_(std::move(onboarded_count_lookup)),
  topic_name_(make_topic_name(topic_namespace)),
  current_state_(vda5050_master_ros2::msg::MasterConnection::STARTING)
{
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  pub_ = node_->create_publisher<vda5050_master_ros2::msg::MasterConnection>(
    topic_name_, qos);

  {
    std::lock_guard<std::mutex> guard(state_mu_);
    publish_current_locked_();
  }

  heartbeat_timer_ = node_->create_wall_timer(
    kHeartbeatInterval, [this] { this->heartbeat_tick_(); });

  VDA5050_INFO(
    "[MasterConnectionPublisher] publishing on {} as master_id={}", topic_name_,
    master_id_);
}

MasterConnectionPublisher::~MasterConnectionPublisher()
{
  try
  {
    set_state(vda5050_master_ros2::msg::MasterConnection::SHUTTING_DOWN);
  }
  catch (...)
  {
    // dtor must not throw
  }
}

void MasterConnectionPublisher::set_state(std::uint8_t new_state)
{
  std::lock_guard<std::mutex> guard(state_mu_);
  if (current_state_ == new_state) return;
  current_state_ = new_state;
  publish_current_locked_();
}

std::uint8_t MasterConnectionPublisher::current_state() const
{
  std::lock_guard<std::mutex> guard(state_mu_);
  return current_state_;
}

void MasterConnectionPublisher::heartbeat_tick_()
{
  std::lock_guard<std::mutex> guard(state_mu_);
  publish_current_locked_();
}

void MasterConnectionPublisher::publish_current_locked_()
{
  vda5050_master_ros2::msg::MasterConnection msg;
  msg.stamp = node_->now();
  msg.state = current_state_;
  msg.master_id = master_id_;
  msg.master_version = master_version_;
  msg.broker_connected =
    broker_connected_lookup_ ? broker_connected_lookup_() : false;
  msg.onboarded_agv_count =
    onboarded_count_lookup_ ? onboarded_count_lookup_() : 0u;
  pub_->publish(msg);
}

}  // namespace vda5050_master_ros2
