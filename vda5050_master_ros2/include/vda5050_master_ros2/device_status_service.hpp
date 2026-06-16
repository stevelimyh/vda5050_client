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

#ifndef VDA5050_MASTER_ROS2__DEVICE_STATUS_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__DEVICE_STATUS_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_master_ros2/srv/get_device_status.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// DeviceStatusService (Task #48 — service half).
// =============================================================================
//
// Synchronous ROS 2 service that returns the master's cached
// State / Connection / Factsheet snapshot for one AGV. Companion to
// DeviceStatusPublisher: where the publisher streams each cached-message
// update, this service answers "what's the current cached truth for AGV
// X right now?" — useful for FMS dashboard refresh after restart, ad-hoc
// diagnostics, and "is AGV X reachable" lookups without subscribing to
// the topic stream.
//
// Service name: /<topic_namespace>/get_device_status
//   Default <topic_namespace> = "vda5050_master" (matches publisher).
//   Configurable via constructor for multi-instance deployments.
//
// **Atomic snapshot**: the callback uses `AGV::get_status_snapshot()`
// which acquires `data_mutex_` once and returns all six cached fields
// (State, Connection, Factsheet + their three timestamps). Sequential
// `get_last_*()` getters would have allowed cache drift between calls
// — observable to the FMS as e.g. "state=driving + connection=BROKEN"
// in the same response, which is wrong.
//
// **AGV lookup**: takes a `std::function<shared_ptr<AGV>(mfg, serial)>`
// rather than a raw `VDA5050Master*`. Decouples the service for unit
// tests (a small map+lambda is enough) and dodges Master ownership
// constraints. `nullptr` from the lookup is interpreted as
// AGV_NOT_ONBOARDED.
//
// **Status enum** (in vda5050_master/srv/GetDeviceStatus):
//   SUCCESS         — at least one of state/connection observed.
//   INVALID_REQUEST — empty manufacturer or serial_number.
//   AGV_NOT_ONBOARDED — lookup returned nullptr.
//   AGV_SILENT      — onboarded but no state OR connection received.
// `has_factsheet=false` alone does NOT block SUCCESS — the Factsheet is
// request/response, not heartbeat.

class DeviceStatusService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "get_device_status";

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
  ///                        "vda5050_master" (matches the
  ///                        DeviceStatusPublisher convention).
  DeviceStatusService(
    rclcpp::Node::SharedPtr node, AgvLookup agv_lookup,
    const std::string& topic_namespace = "vda5050_master");

  ~DeviceStatusService() = default;
  DeviceStatusService(const DeviceStatusService&) = delete;
  DeviceStatusService& operator=(const DeviceStatusService&) = delete;
  DeviceStatusService(DeviceStatusService&&) = delete;
  DeviceStatusService& operator=(DeviceStatusService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name. Useful for tests +
  ///        diagnostics + clients that want to log what they're calling.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using GetDeviceStatus = vda5050_master_ros2::srv::GetDeviceStatus;

  void handle_request(
    const std::shared_ptr<GetDeviceStatus::Request> request,
    std::shared_ptr<GetDeviceStatus::Response> response);

  // Build the service name from namespace + leaf.
  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  AgvLookup agv_lookup_;
  std::string service_name_;
  rclcpp::Service<GetDeviceStatus>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__DEVICE_STATUS_SERVICE_HPP_
