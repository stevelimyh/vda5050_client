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

#ifndef VDA5050_MASTER_ROS2__ORDER_STATUS_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__ORDER_STATUS_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_master_ros2/srv/get_order_status.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OrderStatusService (Task #47 — service half).
// =============================================================================
//
// Synchronous ROS 2 service that returns the master's atomic order-status
// snapshot for one AGV. Companion to OrderStatusPublisher.
//
// Service name: /<topic_namespace>/get_order_status
//   Default <topic_namespace> = "vda5050_master".
//
// **Atomic snapshot**: callback uses `AGV::get_order_status_bundle()`
// which takes `data_mutex_` then queries OrderLifecycleManager,
// giving microsecond-class drift versus the executor-tick drift that
// would occur with separate `get_last_state` + `active_order_snapshot`
// calls.
//
// **AGV lookup**: takes a `std::function<shared_ptr<AGV>(mfg, serial)>`
// (typically wraps VDA5050Master::get_agv). nullptr → AGV_NOT_ONBOARDED.
//
// **Status enum** (in vda5050_master/srv/GetOrderStatus):
//   SUCCESS         — order_status payload populated; phase=NO_ORDER is
//                     a valid SUCCESS, not a service-delivery error.
//   INVALID_REQUEST — empty manufacturer or serial_number.
//   AGV_NOT_ONBOARDED — lookup returned nullptr.
//   AGV_SILENT      — onboarded but no State received AND no active
//                     order tracked — nothing meaningful to report.

class OrderStatusService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "get_order_status";

  /// AGV lookup callable. Returns nullptr when no AGV with the given
  /// {manufacturer, serial_number} is currently onboarded.
  using AgvLookup = std::function<std::shared_ptr<vda5050_core::master::AGV>(
    const std::string& manufacturer, const std::string& serial_number)>;

  /// Active-assignment lookup. Returns the master's recorded
  /// assignment_id for the AGV — or empty string when none. Used to
  /// populate the correlation field in the OrderStatus payload.
  using AssignmentLookup = std::function<std::string(
    const std::string& manufacturer, const std::string& serial_number)>;

  /// \brief Construct.
  /// \param node              ROS 2 node hosting the service. Must
  ///                          outlive this object.
  /// \param agv_lookup        Lookup callable. Typically wraps
  ///                          VDA5050Master::get_agv(...).
  /// \param assignment_lookup Active-assignment lookup. Typically wraps
  ///                          VDA5050Master::get_active_assignment_id.
  ///                          May be null — service will treat all
  ///                          assignments as empty.
  /// \param topic_namespace   Prefix for the service name. Defaults to
  ///                          "vda5050_master".
  OrderStatusService(
    rclcpp::Node::SharedPtr node, AgvLookup agv_lookup,
    AssignmentLookup assignment_lookup = nullptr,
    const std::string& topic_namespace = "vda5050_master");

  ~OrderStatusService() = default;
  OrderStatusService(const OrderStatusService&) = delete;
  OrderStatusService& operator=(const OrderStatusService&) = delete;
  OrderStatusService(OrderStatusService&&) = delete;
  OrderStatusService& operator=(OrderStatusService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using GetOrderStatus = vda5050_master_ros2::srv::GetOrderStatus;

  void handle_request(
    const std::shared_ptr<GetOrderStatus::Request> request,
    std::shared_ptr<GetOrderStatus::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  AgvLookup agv_lookup_;
  AssignmentLookup assignment_lookup_;
  std::string service_name_;
  rclcpp::Service<GetOrderStatus>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ORDER_STATUS_SERVICE_HPP_
