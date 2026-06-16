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

#ifndef VDA5050_MASTER_ROS2__VDA5050_MASTER_ROS2_HPP_
#define VDA5050_MASTER_ROS2__VDA5050_MASTER_ROS2_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_master_ros2/assign_order_request_subscriber.hpp"
#include "vda5050_master_ros2/assignment_result_publisher.hpp"
#include "vda5050_master_ros2/device_status_publisher.hpp"
#include "vda5050_master_ros2/device_status_service.hpp"
#include "vda5050_master_ros2/discard_mode_cancelled_queue_service.hpp"
#include "vda5050_master_ros2/fleet_roster_subscriber.hpp"
#include "vda5050_master_ros2/get_loaded_map_service.hpp"
#include "vda5050_master_ros2/get_master_broker_status_service.hpp"
#include "vda5050_master_ros2/get_pose_view_service.hpp"
#include "vda5050_master_ros2/instant_actions_send_service.hpp"
#include "vda5050_master_ros2/master_connection_publisher.hpp"
#include "vda5050_master_ros2/offboard_agv_batch_service.hpp"
#include "vda5050_master_ros2/offboard_agv_service.hpp"
#include "vda5050_master_ros2/onboard_agv_batch_service.hpp"
#include "vda5050_master_ros2/onboard_agv_service.hpp"
#include "vda5050_master_ros2/order_send_service.hpp"
#include "vda5050_master_ros2/order_status_publisher.hpp"
#include "vda5050_master_ros2/order_status_service.hpp"
#include "vda5050_master_ros2/pose_view_publisher.hpp"
#include "vda5050_master_ros2/resume_mode_cancelled_queue_service.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// VDA5050MasterROS2 (ROS 2 Endpoints epic — first commit, Task #48).
// =============================================================================
//
// Opt-in subclass of VDA5050Master that enables ROS 2 endpoint
// publishing. FMS deployments that want ROS 2 integration construct
// `VDA5050MasterROS2` instead of `VDA5050Master`. Pure C++ users
// continue to use `VDA5050Master` and pay nothing for ROS 2.
//
// **Currently shipped (#48 + #47 + #45 + #46)**:
//   * Device Status publisher — per-AGV State / Connection / Factsheet
//     topics, fired from the existing `on_state` / `on_connection` /
//     `on_factsheet` virtual hooks.
//   * Device Status service — `/{namespace}/get_device_status`
//     synchronous query returning a coherent (single-mutex) snapshot
//     of the master's cached state for one AGV.
//   * Order Status publisher — per-AGV `/order_status` topic, fired
//     from `on_state`. Carries master's lifecycle phase, order id /
//     update id, base/horizon node counts, action_states, errors.
//   * Order Status service — `/{namespace}/get_order_status`
//     synchronous query backed by an atomic State + lifecycle bundle.
//   * Order Send service — `/{namespace}/assign_order` synchronous
//     dispatch entry point. A single service handles BOTH new orders
//     (#45) and order updates (#46) per VDA5050 §6.6.2 semantics.
//     Wraps the existing `master.assign_order(mfg, serial, order)`
//     pre-flight + queue handoff. Returns AssignmentDecision +
//     diagnostic errors[] synchronously.
//   * InstantActions Send service —
//     `/{namespace}/assign_instant_actions` synchronous dispatch entry
//     point for ALL instant actions (predefined + custom). Wraps the
//     existing `master.assign_instant_actions(mfg, serial, actions)`
//     pre-flight + action-conflict detection + §6.10.6 mode gate
//     with §6.8.1 allowlist exemption. Returns InstantActionDecision
//     + diagnostic errors[] synchronously.
//   * Onboard AGV service — `/{namespace}/onboard_agv` synchronous
//     fleet-membership add. Wraps `master.onboard_agv(...)`. Idempotent;
//     returns ALREADY_ONBOARDED for re-onboard.
//   * Offboard AGV service — `/{namespace}/offboard_agv` synchronous
//     fleet-membership remove. Wraps `master.offboard_agv(...)`.
//     Idempotent; returns NOT_ONBOARDED for missing AGV.
//   * GetLoadedMap service — `/{namespace}/get_loaded_map` synchronous
//     query returning the master's currently-loaded topology map
//     metadata + per-AGV factsheet-alignment summary. Operator
//     diagnostics — answers "what map is the master running with, and
//     is every AGV compatible?" without log scraping.
//
// **Override chaining**: each `on_*` override calls
// `VDA5050Master::on_*(...)` after publishing so any further FMS
// subclass override semantics are preserved. FMS that subclass
// VDA5050MasterROS2 should call `VDA5050MasterROS2::on_*(...)` from
// their override to keep ROS 2 publishing intact.

class VDA5050MasterROS2 : public vda5050_core::master::VDA5050Master
{
public:
  /// Default pose_view publish rate (Hz) when not otherwise configured.
  static constexpr double kDefaultPoseViewRateHz = 1.0;
  /// Upper bound on the configurable pose_view rate (Hz); rejected above.
  static constexpr double kMaxPoseViewRateHz = 1000.0;

  /// \brief Construct.
  /// \param mqtt_client      Same MQTT client passed to base; required.
  /// \param ros2_node        ROS 2 node hosting publishers.
  ///                         Caller spins. Must outlive this object.
  /// \param topic_namespace  Prefix for per-AGV ROS 2 topics. Default
  ///                         "vda5050_master".
  /// \param master_id        Identity advertised on MasterConnection.
  ///                         Empty falls back to a hostname-pid string
  ///                         so multi-master deployments get distinct
  ///                         IDs without configuration.
  /// \param master_version   Software version string on MasterConnection.
  /// \param pose_view_rate_hz Fixed publish rate (Hz) for the per-AGV
  ///                          pose_view stream. Must be in
  ///                          (0, kMaxPoseViewRateHz]; out-of-range throws.
  VDA5050MasterROS2(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client,
    rclcpp::Node::SharedPtr ros2_node,
    const std::string& topic_namespace =
      DeviceStatusPublisher::kDefaultNamespace,
    const std::string& master_id = "",
    const std::string& master_version = "vda5050_master_ros2",
    double pose_view_rate_hz = kDefaultPoseViewRateHz);

  ~VDA5050MasterROS2() override;

  VDA5050MasterROS2(const VDA5050MasterROS2&) = delete;
  VDA5050MasterROS2& operator=(const VDA5050MasterROS2&) = delete;
  VDA5050MasterROS2(VDA5050MasterROS2&&) = delete;
  VDA5050MasterROS2& operator=(VDA5050MasterROS2&&) = delete;

  /// Read access for tests + advanced users.
  DeviceStatusPublisher& device_status_publisher()
  {
    return *device_status_;
  }

  /// Read access to the GetDeviceStatus service (test + diagnostics).
  DeviceStatusService& device_status_service()
  {
    return *device_status_service_;
  }

  /// Read access to the OrderStatus publisher (test + diagnostics).
  OrderStatusPublisher& order_status_publisher()
  {
    return *order_status_publisher_;
  }

  /// Read access to the GetOrderStatus service (test + diagnostics).
  OrderStatusService& order_status_service()
  {
    return *order_status_service_;
  }

  /// Read access to the PoseView publisher (test + diagnostics).
  PoseViewPublisher& pose_view_publisher()
  {
    return *pose_view_publisher_;
  }

  /// Read access to the GetPoseView service (test + diagnostics).
  GetPoseViewService& get_pose_view_service()
  {
    return *get_pose_view_service_;
  }

  /// Read access to the AssignOrder service (test + diagnostics).
  OrderSendService& order_send_service()
  {
    return *order_send_service_;
  }

  /// Read access to the AssignInstantActions service (test + diagnostics).
  InstantActionsSendService& instant_actions_send_service()
  {
    return *instant_actions_send_service_;
  }

  /// Read access to the OnboardAGV service (single).
  OnboardAGVService& onboard_agv_service()
  {
    return *onboard_agv_service_;
  }

  /// Read access to the OnboardAGVBatch service (batch).
  OnboardAGVBatchService& onboard_agv_batch_service()
  {
    return *onboard_agv_batch_service_;
  }

  /// Read access to the OffboardAGV service (single).
  OffboardAGVService& offboard_agv_service()
  {
    return *offboard_agv_service_;
  }

  /// Read access to the OffboardAGVBatch service (batch).
  OffboardAGVBatchService& offboard_agv_batch_service()
  {
    return *offboard_agv_batch_service_;
  }

  /// Read access to the GetLoadedMap service (test + diagnostics).
  GetLoadedMapService& get_loaded_map_service()
  {
    return *get_loaded_map_service_;
  }

  /// Read access to the ResumeModeCancelledQueue service (Task #24).
  ResumeModeCancelledQueueService& resume_mode_cancelled_queue_service()
  {
    return *resume_mode_cancelled_queue_service_;
  }

  /// Read access to the DiscardModeCancelledQueue service (Task #24).
  DiscardModeCancelledQueueService& discard_mode_cancelled_queue_service()
  {
    return *discard_mode_cancelled_queue_service_;
  }

  /// Read access to the GetMasterBrokerStatus service (Task #73).
  GetMasterBrokerStatusService& get_master_broker_status_service()
  {
    return *get_master_broker_status_service_;
  }

  /// Read access to the FleetRoster reconciler (test + diagnostics).
  FleetRosterSubscriber& fleet_roster_subscriber()
  {
    return *fleet_roster_subscriber_;
  }

  /// Read access to the MasterConnection publisher (test + diagnostics).
  MasterConnectionPublisher& master_connection_publisher()
  {
    return *master_connection_publisher_;
  }

  /// The master_id resolved at construction (deployed-configured or
  /// hostname-pid fallback).
  const std::string& master_id() const
  {
    return master_id_;
  }

  // ============================================================================
  // VDA5050Master extension callback overrides
  // ============================================================================
  //
  // These intercept the raw cached-message arrivals and publish to ROS 2
  // before chaining to the base implementation (default empty). Further
  // subclasses must call VDA5050MasterROS2::on_*(...) to keep publishing.

  void on_state(
    const std::string& agv_id,
    const vda5050_core::types::State& state) override;
  void on_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection) override;
  void on_factsheet(
    const std::string& agv_id,
    const vda5050_core::types::Factsheet& factsheet) override;

  // Broker connection callbacks — drive MasterConnection state.
  void on_broker_disconnected() override;
  void on_broker_reconnected() override;

private:
  // agv_id is "{manufacturer}/{serial_number}" (per master.cpp:228).
  // Split it back for topic naming.
  static std::pair<std::string, std::string> split_agv_id(
    const std::string& agv_id);

  // Timer callback: publish a fused PoseView for every onboarded AGV.
  void publish_pose_views();

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<DeviceStatusPublisher> device_status_;
  std::unique_ptr<DeviceStatusService> device_status_service_;
  std::unique_ptr<OrderStatusPublisher> order_status_publisher_;
  std::unique_ptr<OrderStatusService> order_status_service_;
  std::unique_ptr<PoseViewPublisher> pose_view_publisher_;
  std::unique_ptr<GetPoseViewService> get_pose_view_service_;
  std::unique_ptr<OrderSendService> order_send_service_;
  std::unique_ptr<InstantActionsSendService> instant_actions_send_service_;
  std::unique_ptr<OnboardAGVService> onboard_agv_service_;
  std::unique_ptr<OnboardAGVBatchService> onboard_agv_batch_service_;
  std::unique_ptr<OffboardAGVService> offboard_agv_service_;
  std::unique_ptr<OffboardAGVBatchService> offboard_agv_batch_service_;
  std::unique_ptr<GetLoadedMapService> get_loaded_map_service_;
  std::unique_ptr<ResumeModeCancelledQueueService>
    resume_mode_cancelled_queue_service_;
  std::unique_ptr<DiscardModeCancelledQueueService>
    discard_mode_cancelled_queue_service_;
  std::unique_ptr<GetMasterBrokerStatusService>
    get_master_broker_status_service_;
  // Async-dispatch — created publisher-first so the subscriber can
  // capture a shared reference at construction time.
  std::shared_ptr<AssignmentResultPublisher> assignment_result_publisher_;
  std::unique_ptr<AssignOrderRequestSubscriber>
    assign_order_request_subscriber_;

  // Device Manager integration.
  std::string master_id_;
  std::unique_ptr<MasterConnectionPublisher> master_connection_publisher_;
  std::unique_ptr<FleetRosterSubscriber> fleet_roster_subscriber_;

  // Declared last so it is destroyed first — the timer callback touches the
  // pose_view publisher and the base AGV registry, which must outlive it.
  rclcpp::TimerBase::SharedPtr pose_view_timer_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__VDA5050_MASTER_ROS2_HPP_
