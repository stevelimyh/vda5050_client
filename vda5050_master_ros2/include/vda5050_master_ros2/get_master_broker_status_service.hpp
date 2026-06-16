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

#ifndef VDA5050_MASTER_ROS2__GET_MASTER_BROKER_STATUS_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__GET_MASTER_BROKER_STATUS_SERVICE_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/srv/get_master_broker_status.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// GetMasterBrokerStatusService (Task #73 — operability surface).
// =============================================================================
//
// Synchronous ROS 2 service returning the master's own MQTT-broker
// connection state. Operator diagnostics — answers "is the master itself
// online with its broker?" Distinct from the per-AGV connection state
// surfaced by GetDeviceStatus: an AGV can be ONLINE while the master
// has lost its broker, and vice versa.
//
// Service name: /<topic_namespace>/get_master_broker_status
//   Default <topic_namespace> = "vda5050_master".
//
// **Decoupled from VDA5050Master via a single callable** (StatusLookup)
// so unit tests can supply a lambda without constructing a Master.
// Same pattern as GetLoadedMapService / DeviceStatusService.

class GetMasterBrokerStatusService
{
public:
  static constexpr const char* kServiceLeaf = "get_master_broker_status";

  /// Plain snapshot mirroring `VDA5050Master::BrokerStatusSnapshot`.
  /// Duplicated here (instead of taking a dependency on master.hpp) so
  /// the service class stays self-contained and easily unit-testable.
  struct StatusSnapshot
  {
    bool connected = false;
    std::optional<std::chrono::system_clock::time_point> last_disconnect_at;
    std::uint64_t reconnect_count = 0;
  };

  using StatusLookup = std::function<StatusSnapshot()>;

  GetMasterBrokerStatusService(
    rclcpp::Node::SharedPtr node, StatusLookup status_lookup,
    const std::string& topic_namespace = "vda5050_master");

  ~GetMasterBrokerStatusService() = default;
  GetMasterBrokerStatusService(const GetMasterBrokerStatusService&) = delete;
  GetMasterBrokerStatusService& operator=(const GetMasterBrokerStatusService&) =
    delete;
  GetMasterBrokerStatusService(GetMasterBrokerStatusService&&) = delete;
  GetMasterBrokerStatusService& operator=(GetMasterBrokerStatusService&&) =
    delete;

  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using GetMasterBrokerStatus = vda5050_master_ros2::srv::GetMasterBrokerStatus;

  void handle_request(
    const std::shared_ptr<GetMasterBrokerStatus::Request> request,
    std::shared_ptr<GetMasterBrokerStatus::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  StatusLookup status_lookup_;
  std::string service_name_;
  rclcpp::Service<GetMasterBrokerStatus>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__GET_MASTER_BROKER_STATUS_SERVICE_HPP_
