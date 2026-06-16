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

#ifndef VDA5050_MASTER_ROS2__ORDER_SEND_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__ORDER_SEND_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/assignment_result.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_master_ros2/srv/assign_order.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OrderSendService (Tasks #45 + #46 — single service for both).
// =============================================================================
//
// Synchronous ROS 2 service that lets an FMS dispatch a VDA5050 Order to
// one AGV. Wraps `VDA5050Master::assign_order(mfg, serial, order)` —
// inheriting its 6-step pre-flight (onboarded + ONLINE + AVAILABLE +
// AUTOMATIC + position_initialized + stitcher pre-flight).
//
// **One service handles both new orders AND order updates** — the
// distinction is encoded inside the Order message per VDA5050 v2.0.0
// §6.6.2 (order_id matching active + order_update_id strictly
// increasing). Task #46 is closed by reusing this service.
//
// Service name: /<topic_namespace>/assign_order
//   Default <topic_namespace> = "vda5050_master".
//
// **`ASSIGNED` semantics**: pre-flight passed; the order is queued for
// publish on the master's queue-processor thread. NOT AGV-acknowledged.
// FMS observes AGV-level acceptance via the OrderStatus topic. Async
// validator failures (schema/graph/traversability/PreSend) on the
// queue thread are not surfaced back through this service today.
//
// **Order conversion**: `vda5050_interfaces::msg::Order` →
// `vda5050_core::types::Order` via `internal::from_msg<>` (JSON
// intermediate; ENABLE_ROS2 build flag).
//
// **OrderSender callable**: takes `std::function` (not raw `Master*`)
// to keep the service unit-testable without a full Master instance.
// VDA5050MasterROS2 wraps `master.assign_order` via a lambda.

class OrderSendService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "assign_order";

  /// Order-dispatch callable. Typically wraps
  /// VDA5050Master::assign_order(...).
  using OrderSender = std::function<vda5050_core::master::AssignmentResult(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order)>;

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the service. Must outlive
  ///                        this object.
  /// \param sender          Order-dispatch callable. Typically wraps
  ///                        VDA5050Master::assign_order(...).
  /// \param topic_namespace Prefix for the service name. Defaults to
  ///                        "vda5050_master".
  OrderSendService(
    rclcpp::Node::SharedPtr node, OrderSender sender,
    const std::string& topic_namespace = "vda5050_master");

  ~OrderSendService() = default;
  OrderSendService(const OrderSendService&) = delete;
  OrderSendService& operator=(const OrderSendService&) = delete;
  OrderSendService(OrderSendService&&) = delete;
  OrderSendService& operator=(OrderSendService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using AssignOrder = vda5050_master_ros2::srv::AssignOrder;

  void handle_request(
    const std::shared_ptr<AssignOrder::Request> request,
    std::shared_ptr<AssignOrder::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OrderSender sender_;
  std::string service_name_;
  rclcpp::Service<AssignOrder>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ORDER_SEND_SERVICE_HPP_
