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

// example_master — library-backed sample VDA5050 master process.
//
// Demonstrates the recommended FMS-integration pattern:
//   1. Subclass `VDA5050MasterROS2` to inherit MQTT transport, validators,
//      vda5050_core::master::AGV management, order lifecycle, AND 10 ROS 2 service endpoints +
//      4 topic publishers — all wired in by the parent's constructor.
//   2. Override `on_*` virtuals for observability and FMS-side reactions.
//      Each override forwards to `Parent::on_*` first so the inherited
//      ROS 2 publishing chain stays intact.
//   3. Load a topology map at startup (drives the traversability and
//      factsheet-alignment validators).
//   4. Connect, then spin the ROS 2 executor. AGVs join at runtime via
//      `/<namespace>/onboard_agv`; FMS dispatches orders via
//      `/<namespace>/assign_order` and instant actions via
//      `/<namespace>/assign_instant_actions`.
//
// Pair with `mock_fms` for a full scripted end-to-end demo.
//
// Run:
//   ros2 run vda5050_master example_master
//
// Environment overrides (defaults shown):
//   VDA5050_BROKER_URL=tcp://localhost:1883
//   VDA5050_MASTER_NAMESPACE=vda5050_master
//   VDA5050_MAP_PATH=<package_share>/sample_data/sample_map.json

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/error_level.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_master_ros2/vda5050_master_ros2.hpp"

namespace {

std::atomic_bool g_running{true};

void signal_handler(int signal)
{
  VDA5050_INFO(
    "Signal [{}] received. Shutting down example_master ...", signal);
  g_running = false;
}

std::string env_or(const char* var, const std::string& fallback)
{
  const char* v = std::getenv(var);
  return (v && *v) ? std::string(v) : fallback;
}

// Stringify helpers — short labels for log readability.

const char* to_label(vda5050_core::types::OperatingMode m)
{
  switch (m)
  {
    case vda5050_core::types::OperatingMode::AUTOMATIC:
      return "AUTOMATIC";
    case vda5050_core::types::OperatingMode::SEMIAUTOMATIC:
      return "SEMIAUTOMATIC";
    case vda5050_core::types::OperatingMode::MANUAL:
      return "MANUAL";
    case vda5050_core::types::OperatingMode::SERVICE:
      return "SERVICE";
    case vda5050_core::types::OperatingMode::TEACHIN:
      return "TEACHIN";
  }
  return "?";
}

const char* to_label(vda5050_core::types::ConnectionState s)
{
  switch (s)
  {
    case vda5050_core::types::ConnectionState::ONLINE:
      return "ONLINE";
    case vda5050_core::types::ConnectionState::OFFLINE:
      return "OFFLINE";
    case vda5050_core::types::ConnectionState::CONNECTIONBROKEN:
      return "CONNECTIONBROKEN";
  }
  return "?";
}

const char* to_label(vda5050_core::types::ErrorLevel l)
{
  switch (l)
  {
    case vda5050_core::types::ErrorLevel::WARNING:
      return "WARNING";
    case vda5050_core::types::ErrorLevel::FATAL:
      return "FATAL";
  }
  return "?";
}

}  // namespace

// =============================================================================
// ExampleMaster — minimal FMS-side subclass of VDA5050MasterROS2.
// =============================================================================
//
// Every `on_*` override:
//   1. Calls `Parent::on_*(...)` FIRST to preserve the inherited ROS 2
//      publishing chain (state / connection / factsheet / order_status
//      topics — driven by the parent's overrides).
//   2. Then logs a one-line structured summary for operator visibility.
//
// A production FMS would replace these log calls with real policy
// (alerting, dispatching new orders, halting the fleet, etc.). The
// pattern is identical — override and forward.

class ExampleMaster : public vda5050_master_ros2::VDA5050MasterROS2
{
public:
  using Parent = vda5050_master_ros2::VDA5050MasterROS2;
  using Parent::Parent;  // inherit constructor

  // --- Raw-message hooks --------------------------------------------------

  void on_state(
    const std::string& agv_id, const vda5050_core::types::State& state) override
  {
    Parent::on_state(agv_id, state);
    VDA5050_INFO(
      "[STATE] {} order={}/{} last_node={} driving={} paused={} errors={}",
      agv_id, state.order_id, state.order_update_id, state.last_node_id,
      state.driving, state.paused.value_or(false), state.errors.size());
  }

  void on_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection) override
  {
    Parent::on_connection(agv_id, connection);
    VDA5050_INFO(
      "[CONN] {} state={}", agv_id, to_label(connection.connection_state));
  }

  void on_factsheet(
    const std::string& agv_id,
    const vda5050_core::types::Factsheet& factsheet) override
  {
    Parent::on_factsheet(agv_id, factsheet);
    VDA5050_INFO(
      "[FACTSHEET] {} series={}", agv_id,
      factsheet.type_specification.series_name);
  }

  // --- Edge-detected event hooks -----------------------------------------

  void on_mode_changed(
    const std::string& agv_id, vda5050_core::types::OperatingMode new_mode,
    vda5050_core::types::OperatingMode prev_mode) override
  {
    Parent::on_mode_changed(agv_id, new_mode, prev_mode);
    VDA5050_INFO(
      "[MODE] {} {} -> {}", agv_id, to_label(prev_mode), to_label(new_mode));

    // Demonstrate the recommended return-to-AUTOMATIC pattern from
    // master.hpp:484-491. On the return edge, FMS commits to either
    // resume (preserve queue order) or discard. We choose resume here
    // since the demo wants order continuity. A production FMS could
    // pick based on time-since-leave, current fleet state, etc.
    if (
      new_mode == vda5050_core::types::OperatingMode::AUTOMATIC &&
      prev_mode != vda5050_core::types::OperatingMode::AUTOMATIC)
    {
      // Internal vda5050_core::master::AGV-id format is "manufacturer/serial_number" — split
      // on '/' to call the public 2-arg get_agv().
      const auto slash = agv_id.find('/');
      if (slash == std::string::npos) return;
      const std::string mfg = agv_id.substr(0, slash);
      const std::string sn = agv_id.substr(slash + 1);
      auto agv = get_agv(mfg, sn);
      if (agv)
      {
        const auto [orders_n, actions_n] = agv->resume_mode_cancelled_queue();
        if (orders_n + actions_n > 0)
        {
          VDA5050_INFO(
            "[MODE] {} resumed {} order(s) + {} instant action(s)", agv_id,
            orders_n, actions_n);
        }
      }
    }
  }

  void on_errors_appeared(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Error>& new_errors) override
  {
    Parent::on_errors_appeared(agv_id, new_errors);
    for (const auto& e : new_errors)
    {
      VDA5050_WARN(
        "[ERROR] {} appeared: type={} level={} description={}", agv_id,
        e.error_type, to_label(e.error_level),
        e.error_description.value_or(""));
    }
  }

  // --- State-timeout -----------------------------------------------------

  void on_state_timeout(const std::string& agv_id) override
  {
    Parent::on_state_timeout(agv_id);
    VDA5050_WARN(
      "[TIMEOUT] {} state heartbeat lost (no State in >30s)", agv_id);
  }

  void on_state_resumed(const std::string& agv_id) override
  {
    Parent::on_state_resumed(agv_id);
    VDA5050_INFO("[RECOVERED] {} state heartbeat resumed", agv_id);
  }

  // --- Connection lifecycle ----------------------------------------------

  void on_connect(const std::string& agv_id) override
  {
    Parent::on_connect(agv_id);
    VDA5050_INFO("[CONNECTED] {} came ONLINE", agv_id);
  }

  void on_offline(const std::string& agv_id) override
  {
    Parent::on_offline(agv_id);
    VDA5050_INFO("[OFFLINE] {} announced graceful OFFLINE", agv_id);
  }

  void on_connection_broken(const std::string& agv_id) override
  {
    Parent::on_connection_broken(agv_id);
    VDA5050_WARN(
      "[BROKEN] {} unexpectedly disconnected (last-will fired)", agv_id);
  }

  // --- Master-broker observability ---------------------------------------

  void on_broker_disconnected() override
  {
    Parent::on_broker_disconnected();
    VDA5050_ERROR(
      "[BROKER] master lost MQTT broker connection — orders cannot flow");
  }

  void on_broker_reconnected() override
  {
    Parent::on_broker_reconnected();
    VDA5050_INFO("[BROKER] master reconnected to MQTT broker");
  }
};

// =============================================================================
// main
// =============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  const std::string broker =
    env_or("VDA5050_BROKER_URL", "tcp://localhost:1883");
  const std::string ns = env_or("VDA5050_MASTER_NAMESPACE", "vda5050_master");
  std::string map_path = env_or("VDA5050_MAP_PATH", "");
  if (map_path.empty())
  {
    try
    {
      map_path =
        ament_index_cpp::get_package_share_directory("vda5050_master_ros2") +
        "/sample_data/sample_map.json";
    }
    catch (const std::exception& e)
    {
      VDA5050_WARN(
        "Could not resolve default map path ({}). Running without map.",
        e.what());
      map_path.clear();
    }
  }

  double pose_view_rate_hz = 1.0;
  {
    const std::string rate_str = env_or("VDA5050_POSE_VIEW_RATE_HZ", "1.0");
    try
    {
      pose_view_rate_hz = std::stod(rate_str);
    }
    catch (const std::exception&)
    {
      VDA5050_WARN(
        "Invalid VDA5050_POSE_VIEW_RATE_HZ='{}'; using 1.0 Hz", rate_str);
      pose_view_rate_hz = 1.0;
    }
  }

  VDA5050_INFO("ExampleMaster starting:");
  VDA5050_INFO("  Broker:    {}", broker);
  VDA5050_INFO("  Namespace: {}", ns);
  VDA5050_INFO(
    "  vda5050_core::master::Map path:  {}",
    map_path.empty() ? "(none — pre-send map check will reject orders)"
                     : map_path);
  VDA5050_INFO("  PoseView:  {} Hz", pose_view_rate_hz);

  // Per-PID client_id avoids broker-side session-takeover races when
  // master is restarted within Paho's keepalive window (~60s). With a
  // fixed id, the new connection forces the broker to evict the old
  // session, and subscriptions can be dropped mid-handover.
  auto mqtt = vda5050_core::transport::create_default_client_shared(
    broker, fmt::format("example_master-{}", ::getpid()));
  auto node = std::make_shared<rclcpp::Node>("example_master");
  auto master = std::make_shared<ExampleMaster>(
    mqtt, node, ns, "", "vda5050_master_ros2", pose_view_rate_hz);

  if (!map_path.empty())
  {
    const auto result = master->load_map_from_config(map_path);
    if (result)
    {
      VDA5050_INFO(
        "Loaded map id={} version={} ({} nodes, {} edges)",
        result.map->info.map_id, result.map->info.map_version,
        result.map->nodes.size(), result.map->edges.size());
    }
    else
    {
      VDA5050_ERROR(
        "vda5050_core::master::Map load from {} failed ({} error(s)). Running "
        "without map.",
        map_path, result.errors.size());
    }
  }

  master->connect();
  VDA5050_INFO(
    "ExampleMaster ready (namespace=/{0}). "
    "Onboard AGVs via /{0}/onboard_agv; dispatch orders via /{0}/assign_order.",
    ns);

  // MultiThreadedExecutor: lets service responses get delivered on
  // a different thread than the one chewing on MQTT-driven callbacks
  // (state/connection/heartbeat). Single-threaded starves the service
  // response under heavy AGV traffic.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  while (g_running && rclcpp::ok())
  {
    exec.spin_some(std::chrono::milliseconds(50));
  }

  VDA5050_INFO("Shutting down example_master ...");
  master->disconnect();
  rclcpp::shutdown();
  return 0;
}
