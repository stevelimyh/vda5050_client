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

#include "vda5050_master_ros2/order_status_publisher.hpp"

#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master_ros2/internal/ros2_topic_naming.hpp"

namespace vda5050_master_ros2 {
OrderStatusPublisher::OrderStatusPublisher(
  rclcpp::Node::SharedPtr node, const std::string& topic_namespace)
: node_(std::move(node)), namespace_(topic_namespace)
{
}

std::string OrderStatusPublisher::agv_id(
  const std::string& manufacturer, const std::string& serial_number)
{
  return manufacturer + "/" + serial_number;
}

std::string OrderStatusPublisher::order_status_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::string topic = "/";
  if (!namespace_.empty())
  {
    topic += namespace_;
    topic += "/";
  }
  topic += internal::to_ros2_topic_segment(manufacturer);
  topic += "/";
  topic += internal::to_ros2_topic_segment(serial_number);
  topic += "/order_status";
  return topic;
}

bool OrderStatusPublisher::has_publisher_for(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return publishers_.count(agv_id(manufacturer, serial_number)) != 0;
}

rclcpp::Publisher<vda5050_master_ros2::msg::OrderStatus>::SharedPtr
OrderStatusPublisher::ensure_publisher_locked(
  const std::string& manufacturer, const std::string& serial_number)
{
  const std::string id = agv_id(manufacturer, serial_number);
  auto it = publishers_.find(id);
  if (it != publishers_.end()) return it->second;

  const std::string topic = order_status_topic(manufacturer, serial_number);
  auto pub = node_->create_publisher<vda5050_master_ros2::msg::OrderStatus>(
    topic, kQosDepth);

  if (
    internal::needs_topic_sanitization(manufacturer) ||
    internal::needs_topic_sanitization(serial_number))
  {
    VDA5050_INFO(
      "[OrderStatusPublisher] sanitized ROS 2 segment for AGV {} -> topic {} "
      "(ROS 2 names cannot start with a digit; '_' was prepended)",
      id, topic);
  }
  VDA5050_INFO(
    "[OrderStatusPublisher] created publisher for {} on topic {}", id, topic);

  return publishers_.emplace(id, std::move(pub)).first->second;
}

void OrderStatusPublisher::publish_order_status(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_master_ros2::msg::OrderStatus& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto pub = ensure_publisher_locked(manufacturer, serial_number);
  pub->publish(msg);
}

void OrderStatusPublisher::remove_agv(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishers_.erase(agv_id(manufacturer, serial_number));
}

}  // namespace vda5050_master_ros2
