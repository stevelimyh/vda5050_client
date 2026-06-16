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

// mock_client — single-AGV pure-VDA5050 stub for manual integration tests
// against example_master. Talks MQTT only — no rclcpp, no ROS 2 surface.
//
// Pattern mirrors Saurabh's `mock_master_control.cpp`, role-inverted: this is
// the AGV side. Uses the same MQTT + serialization stack the master uses
// (vda5050_core::transport + ::execution::ProtocolAdapter).
//
// Run:
//   ros2 run vda5050_master_ros2 mock_client
//       --serial MOCK001 --mode AUTOMATIC --tick-ms 1000
//
// Signals:
//   SIGINT / SIGTERM   graceful: publish Connection{OFFLINE}, disconnect
//   SIGUSR1            toggle operating_mode AUTOMATIC <-> MANUAL
//   SIGUSR2            toggle paused
//
// Scenarios (--scenario):
//   idle             default; standard healthy AGV
//   uninit-pose      AgvPosition.position_initialized = false
//   no-factsheet     skip the initial Factsheet publish
//   malformed-state  emit one bad-JSON State at startup, then resume normal

#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/agv_class.hpp"
#include "vda5050_core/types/agv_kinematic.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/e_stop.hpp"
#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/edge_state.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/node_position.hpp"
#include "vda5050_core/types/node_state.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/type_specification.hpp"

namespace {

using vda5050_core::execution::ProtocolAdapter;
using vda5050_core::transport::MqttClientInterface;
using vda5050_core::types::Action;
using vda5050_core::types::ActionState;
using vda5050_core::types::ActionStatus;
using vda5050_core::types::AGVClass;
using vda5050_core::types::AGVKinematic;
using vda5050_core::types::AGVPosition;
using vda5050_core::types::BatteryState;
using vda5050_core::types::BlockingType;
using vda5050_core::types::Connection;
using vda5050_core::types::ConnectionState;
using vda5050_core::types::EdgeState;
using vda5050_core::types::EStop;
using vda5050_core::types::Factsheet;
using vda5050_core::types::Header;
using vda5050_core::types::InstantActions;
using vda5050_core::types::NodePosition;
using vda5050_core::types::NodeState;
using vda5050_core::types::OperatingMode;
using vda5050_core::types::Order;
using vda5050_core::types::SafetyState;
using vda5050_core::types::State;

// VDA5050 has two distinct "version" strings on the wire:
//   - kTopicVersion: appears in MQTT topic paths (uagv/v2/...). The
//     master's `Version` constant ("v2") is what AGV::setup_subscriptions
//     uses, so we must match it to route messages.
//   - kProtocolVersion: appears in JSON header.version. The master's
//     schema validator (SupportedSchemaVersions = {"2.0.0"}) requires
//     this value. We override the adapter-filled header on outbound
//     publishes so messages pass the master's incoming schema check.
constexpr const char* kInterface = "uagv";
constexpr const char* kTopicVersion = "v2";
constexpr const char* kProtocolVersion = "2.0.0";
constexpr int kDefaultTickMs = 1000;

// CLI options. Defaults make a healthy AGV that lands at N0 of the
// example_master sample map (warehouse_floor1, N0 = origin).
struct Cli
{
  std::string broker = "tcp://localhost:1883";
  std::string mfg = "MockMfg";
  std::string serial = "MOCK001";
  std::string map_id = "warehouse_floor1";
  std::string mode_str = "AUTOMATIC";
  std::string scenario = "idle";
  double x0 = 0.0;
  double y0 = 0.0;
  double theta0 = 0.0;
  int tick_ms = kDefaultTickMs;
};

OperatingMode parse_mode(const std::string& s)
{
  if (s == "AUTOMATIC") return OperatingMode::AUTOMATIC;
  if (s == "SEMIAUTOMATIC") return OperatingMode::SEMIAUTOMATIC;
  if (s == "MANUAL") return OperatingMode::MANUAL;
  if (s == "SERVICE") return OperatingMode::SERVICE;
  if (s == "TEACHIN") return OperatingMode::TEACHIN;
  return OperatingMode::AUTOMATIC;
}

const char* mode_label(OperatingMode m)
{
  switch (m)
  {
    case OperatingMode::AUTOMATIC:
      return "AUTOMATIC";
    case OperatingMode::SEMIAUTOMATIC:
      return "SEMIAUTOMATIC";
    case OperatingMode::MANUAL:
      return "MANUAL";
    case OperatingMode::SERVICE:
      return "SERVICE";
    case OperatingMode::TEACHIN:
      return "TEACHIN";
  }
  return "?";
}

void print_usage()
{
  fmt::print(
    "Usage: mock_client [options]\n"
    "  --broker URL       MQTT broker URL (default tcp://localhost:1883)\n"
    "  --mfg STR          Manufacturer (default MockMfg)\n"
    "  --serial STR       Serial number (default MOCK001)\n"
    "  --map-id STR       Map id reported in state (default warehouse_floor1)\n"
    "  --x0 N             Initial x position (default 0.0)\n"
    "  --y0 N             Initial y position (default 0.0)\n"
    "  --theta0 N         Initial orientation [rad] (default 0.0)\n"
    "  --mode M           Initial operating mode (AUTOMATIC|MANUAL|...)\n"
    "  --tick-ms N        Tick / state-publish period (default 1000)\n"
    "  --scenario S       idle|uninit-pose|no-factsheet|malformed-state\n"
    "\n"
    "Signals: SIGINT/TERM = graceful OFFLINE, SIGUSR1 = toggle mode,\n"
    "         SIGUSR2 = toggle paused.\n");
}

Cli parse_cli(int argc, char** argv)
{
  Cli cli;
  for (int i = 1; i < argc; ++i)
  {
    std::string a = argv[i];
    auto next = [&](const char* flag) -> std::string {
      if (i + 1 >= argc)
      {
        fmt::print(stderr, "missing argument for {}\n", flag);
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--broker")
      cli.broker = next("--broker");
    else if (a == "--mfg")
      cli.mfg = next("--mfg");
    else if (a == "--serial")
      cli.serial = next("--serial");
    else if (a == "--map-id")
      cli.map_id = next("--map-id");
    else if (a == "--x0")
      cli.x0 = std::stod(next("--x0"));
    else if (a == "--y0")
      cli.y0 = std::stod(next("--y0"));
    else if (a == "--theta0")
      cli.theta0 = std::stod(next("--theta0"));
    else if (a == "--mode")
      cli.mode_str = next("--mode");
    else if (a == "--tick-ms")
      cli.tick_ms = std::stoi(next("--tick-ms"));
    else if (a == "--scenario")
      cli.scenario = next("--scenario");
    else if (a == "-h" || a == "--help")
    {
      print_usage();
      std::exit(0);
    }
    else
    {
      fmt::print(stderr, "unknown option: {}\n", a);
      print_usage();
      std::exit(2);
    }
  }
  return cli;
}

// Process-level signal plumbing. We only ever set atomics; the tick
// thread reads them on its next wake-up.
std::atomic_bool g_alive{true};
std::atomic_bool g_pending_mode_flip{false};
std::atomic_bool g_pending_pause_flip{false};

void on_signal(int sig)
{
  switch (sig)
  {
    case SIGINT:
    case SIGTERM:
      g_alive = false;
      break;
    case SIGUSR1:
      g_pending_mode_flip = true;
      break;
    case SIGUSR2:
      g_pending_pause_flip = true;
      break;
    default:
      break;
  }
}

// One unified pending-step view of the active order, in sequence-id
// order. The mock walks this list one entry per tick.
struct PendingStep
{
  bool is_node;  // false => edge
  uint32_t sequence_id;
  std::string id;                        // node_id or edge_id
  std::optional<NodePosition> end_pose;  // only for nodes
  std::vector<Action> actions;           // node or edge actions
};

// =============================================================================
// MockClient
// =============================================================================
//
// Threading model — two threads:
//   1. Paho network thread (owned by transport). Delivers on_order and
//      on_instant_actions. These callbacks acquire `state_mtx_`, mutate
//      `active_order_` / `pending_iactions_`, and return. They never
//      publish: that's the tick thread's job.
//   2. Tick thread (this->tick_). Wakes every `cli_.tick_ms` ms,
//      acquires `state_mtx_`, advances the walking state machine,
//      copies a snapshot of `state_` under the lock, releases, then
//      publishes the snapshot.
//
// Locking discipline:
//   - state_mtx_ protects: state_, active_order_, pending_steps_,
//     active_step_actions_, pending_iactions_, factsheet_request_pending_.
//   - NEVER call adapter_->publish(...) while holding state_mtx_:
//     ProtocolAdapter takes its own internal mutex on
//     publish/subscribe; mixed lock order is a deadlock risk.
//   - adapter_ / mqtt_ are set in ctor and never reassigned — safe
//     to read from any thread without the lock.
class MockClient
{
public:
  explicit MockClient(const Cli& cli);
  ~MockClient();

  MockClient(const MockClient&) = delete;
  MockClient& operator=(const MockClient&) = delete;

  void run();

private:
  // Threads / wiring.
  std::shared_ptr<MqttClientInterface> mqtt_;
  std::shared_ptr<ProtocolAdapter> adapter_;
  std::thread tick_;

  // Config (immutable after ctor).
  const Cli cli_;
  const OperatingMode initial_mode_;

  // Per-topic header counters for raw outbound publishes (we bypass
  // ProtocolAdapter::publish to override header.version with the
  // schema-validator-compatible kProtocolVersion).
  std::atomic<uint32_t> conn_header_id_{0};
  std::atomic<uint32_t> state_header_id_{0};
  std::atomic<uint32_t> factsheet_header_id_{0};

  // Synchronized state.
  std::mutex state_mtx_;
  State state_;
  std::optional<Order> active_order_;
  std::deque<PendingStep> pending_steps_;
  std::unordered_map<std::string, int> active_step_actions_;  // id -> ticks
  std::deque<Action> pending_iactions_;
  // Set by drain_instant_actions_locked when a factsheetRequest action
  // is acknowledged; consumed once per tick by tick_loop, which then
  // calls publish_factsheet_minimal() outside the lock. Avoids the
  // earlier scan-action_states-for-FINISHED-factsheetRequest path.
  bool factsheet_request_pending_ = false;

  // Header helper — common to all outbound publishes.
  Header make_header(uint32_t id);

  // Snapshot-publish helpers (callable with NO lock held).
  void publish_state_snapshot(State snapshot);
  void publish_connection(ConnectionState s);
  void publish_factsheet_minimal();
  void publish_raw_state_payload(const std::string& payload);

  // Subscription callbacks (Paho network thread).
  void on_order(Order order, std::optional<vda5050_core::types::Error> err);
  void on_instant_actions(
    InstantActions ia, std::optional<vda5050_core::types::Error> err);

  // Tick-loop helpers (called with state_mtx_ HELD).
  void apply_order_locked(const Order& order);
  void drain_instant_actions_locked();
  void advance_step_locked();

  // Tick loop body.
  void tick_loop();
};

MockClient::MockClient(const Cli& cli)
: cli_(cli), initial_mode_(parse_mode(cli.mode_str))
{
  // --- 1. Seed canonical State -----------------------------------------------
  state_.operating_mode = initial_mode_;
  state_.driving = false;
  state_.paused = false;
  state_.battery_state = BatteryState{};
  state_.battery_state.battery_charge = 95.0;  // healthy charge
  state_.battery_state.charging = false;
  state_.safety_state = SafetyState{};
  state_.safety_state.e_stop = EStop::NONE;
  state_.safety_state.field_violation = false;
  AGVPosition pose;
  pose.x = cli_.x0;
  pose.y = cli_.y0;
  pose.theta = cli_.theta0;
  pose.map_id = cli_.map_id;
  pose.position_initialized = (cli_.scenario != "uninit-pose");
  state_.agv_position = pose;

  // --- 2. Build will payload BEFORE connect ----------------------------------
  // ProtocolAdapter::set_will<T> does NOT exist. The raw mqtt_->set_will
  // is what's actually wired into Paho. The will topic must match what
  // ProtocolAdapter would publish on for Connection.
  mqtt_ = vda5050_core::transport::create_default_client_shared(
    cli_.broker, fmt::format("mock_client-{}", cli_.serial));

  const auto will_topic = fmt::format(
    "{}/{}/{}/{}/connection", kInterface, kTopicVersion, cli_.mfg, cli_.serial);
  Connection will;
  will.header = make_header(0);
  will.connection_state = ConnectionState::CONNECTIONBROKEN;
  nlohmann::json will_json = will;
  mqtt_->set_will(will_topic, will_json.dump(), /*qos=*/1);

  // --- 3. Connect + adapter --------------------------------------------------
  // Adapter is used for INCOMING subscribe<Order>/<InstantActions> only.
  // Topic strings derive from kTopicVersion so they match the master's
  // setup_subscriptions which also uses Version="v2".
  mqtt_->connect();
  adapter_ = ProtocolAdapter::make(
    mqtt_, kInterface, kTopicVersion, cli_.mfg, cli_.serial);

  // --- 4. Subscriptions ------------------------------------------------------
  adapter_->subscribe<Order>(
    [this](Order o, std::optional<vda5050_core::types::Error> e) {
      this->on_order(std::move(o), std::move(e));
    },
    /*qos=*/0);
  adapter_->subscribe<InstantActions>(
    [this](InstantActions ia, std::optional<vda5050_core::types::Error> e) {
      this->on_instant_actions(std::move(ia), std::move(e));
    },
    /*qos=*/0);

  // --- 5. Initial publishes --------------------------------------------------
  // Connection ONLINE (retained, QoS 1 per spec for connection topic).
  publish_connection(ConnectionState::ONLINE);

  // Factsheet first (so master's factsheet_alignment validator has
  // something to compare against on the next Order).
  if (cli_.scenario != "no-factsheet")
  {
    publish_factsheet_minimal();
  }

  // Malformed-state scenario: one bad-JSON state before the loop starts.
  if (cli_.scenario == "malformed-state")
  {
    publish_raw_state_payload("{ this is not valid JSON ");
  }

  // --- 6. Start tick thread --------------------------------------------------
  tick_ = std::thread([this]() { this->tick_loop(); });

  VDA5050_INFO(
    "[mock_client] up: mfg={} serial={} mode={} pose=({:.2f},{:.2f},{:.2f}) "
    "map={} scenario={} tick={}ms",
    cli_.mfg, cli_.serial, mode_label(initial_mode_), cli_.x0, cli_.y0,
    cli_.theta0, cli_.map_id, cli_.scenario, cli_.tick_ms);
}

MockClient::~MockClient()
{
  // Deactivate subscriptions FIRST so any in-flight delivery becomes
  // a no-op at the adapter layer (ProtocolAdapter's active_flags_).
  if (adapter_)
  {
    adapter_->unsubscribe<Order>();
    adapter_->unsubscribe<InstantActions>();
  }
  // Join tick thread.
  if (tick_.joinable())
  {
    tick_.join();
  }
  // Adapter destructs next, then mqtt_ via shared_ptr lifecycle. The
  // mqtt_->disconnect() happens in MqttClientInterface impl dtor.
  if (mqtt_ && mqtt_->connected())
  {
    mqtt_->disconnect();
  }
}

void MockClient::run()
{
  // Main thread blocks until the tick thread exits.
  if (tick_.joinable())
  {
    tick_.join();
    tick_ = std::thread{};
  }
}

// --- publish helpers (NO lock held) -----------------------------------------

Header MockClient::make_header(uint32_t id)
{
  Header h;
  h.header_id = id;
  h.timestamp = std::chrono::system_clock::now();
  h.version = kProtocolVersion;  // "2.0.0" — schema-validator compatible
  h.manufacturer = cli_.mfg;
  h.serial_number = cli_.serial;
  return h;
}

void MockClient::publish_state_snapshot(State snapshot)
{
  // Bypass ProtocolAdapter::publish to set header.version="2.0.0"
  // (the master's schema validator's SupportedSchemaVersions set).
  // ProtocolAdapter would overwrite the header with version="v2".
  snapshot.header = make_header(state_header_id_++);
  const auto topic = fmt::format(
    "{}/{}/{}/{}/state", kInterface, kTopicVersion, cli_.mfg, cli_.serial);
  nlohmann::json j = snapshot;
  if (mqtt_) mqtt_->publish(topic, j.dump(), /*qos=*/0);
}

void MockClient::publish_connection(ConnectionState s)
{
  Connection c;
  c.header = make_header(conn_header_id_++);
  c.connection_state = s;
  const auto topic = fmt::format(
    "{}/{}/{}/{}/connection", kInterface, kTopicVersion, cli_.mfg, cli_.serial);
  nlohmann::json j = c;
  // Connection is retained QoS 1 per VDA5050 §6.14.
  if (mqtt_) mqtt_->publish(topic, j.dump(), /*qos=*/1, /*retain=*/true);
}

void MockClient::publish_factsheet_minimal()
{
  Factsheet fs;
  fs.header = make_header(factsheet_header_id_++);
  // type_specification's enum members carry no default initializer in the
  // struct definition — explicit assignment is required, otherwise the
  // serializer rejects with "Invalid AGVKinematic/AGVClass enum value".
  fs.type_specification.series_name = "MockAGV";
  fs.type_specification.agv_kinematic = AGVKinematic::DIFF;
  fs.type_specification.agv_class = AGVClass::CARRIER;
  fs.type_specification.max_load_mass = 50.0;
  fs.type_specification.localization_types = {"NATURAL"};
  fs.type_specification.navigation_types = {"AUTONOMOUS"};
  fs.physical_parameters.speed_max = 1.5;
  fs.physical_parameters.speed_min = 0.0;
  fs.physical_parameters.acceleration_max = 1.0;
  fs.physical_parameters.deceleration_max = 1.0;
  fs.physical_parameters.width = 0.5;
  fs.physical_parameters.length = 0.7;
  fs.physical_parameters.height_min = 0.1;
  fs.physical_parameters.height_max = 0.5;
  const auto topic = fmt::format(
    "{}/{}/{}/{}/factsheet", kInterface, kTopicVersion, cli_.mfg, cli_.serial);
  nlohmann::json j = fs;
  if (mqtt_) mqtt_->publish(topic, j.dump(), /*qos=*/0);
}

void MockClient::publish_raw_state_payload(const std::string& payload)
{
  // Bypass typed publish to emit malformed bytes — used by the
  // `malformed-state` scenario to exercise master schema validator.
  const auto topic = fmt::format(
    "{}/{}/{}/{}/state", kInterface, kTopicVersion, cli_.mfg, cli_.serial);
  if (mqtt_) mqtt_->publish(topic, payload, /*qos=*/0);
}

// --- subscription callbacks (Paho network thread) ---------------------------

void MockClient::on_order(
  Order order, std::optional<vda5050_core::types::Error> err)
{
  if (err.has_value())
  {
    VDA5050_WARN(
      "[mock_client] Order deserialization error: {}",
      err->error_description.value_or(""));
    return;
  }
  std::lock_guard<std::mutex> lock(state_mtx_);
  apply_order_locked(order);
}

void MockClient::on_instant_actions(
  InstantActions ia, std::optional<vda5050_core::types::Error> err)
{
  if (err.has_value())
  {
    VDA5050_WARN(
      "[mock_client] InstantActions deserialization error: {}",
      err->error_description.value_or(""));
    return;
  }
  std::lock_guard<std::mutex> lock(state_mtx_);
  for (const auto& a : ia.actions)
  {
    pending_iactions_.push_back(a);
    ActionState as;
    as.action_id = a.action_id;
    as.action_type = a.action_type;
    as.action_status = ActionStatus::WAITING;
    state_.action_states.push_back(as);
  }
  VDA5050_INFO(
    "[mock_client] queued {} instant action(s) (total pending {})",
    ia.actions.size(), pending_iactions_.size());
}

// --- tick-loop helpers (state_mtx_ HELD) ------------------------------------

void MockClient::apply_order_locked(const Order& order)
{
  const bool is_update =
    active_order_.has_value() && active_order_->order_id == order.order_id;
  active_order_ = order;
  state_.order_id = order.order_id;
  state_.order_update_id = order.order_update_id;

  // Rebuild pending_steps_ in sequence_id order, interleaving nodes
  // and edges. Skip the first node IF the AGV is already on it
  // (sequence_id 0 + last_node_id matches) on a fresh order — but for
  // the simple mock we walk every step.
  //
  // Horizon (released=false) nodes/edges still appear in
  // state_.node_states/edge_states for master visibility, but are NOT
  // pushed to pending_steps_ — the AGV must park at the last released
  // node and wait for an order update that releases the next segment
  // (VDA5050 v2.0.0 §6.6.4). When an update arrives with same order_id
  // and higher order_update_id, this function rebuilds pending_steps_
  // from the (now-released) items whose sequence_id is past the AGV's
  // current last_node_sequence_id, so previously-walked steps aren't
  // re-walked.
  const uint32_t walked_through = is_update ? state_.last_node_sequence_id : 0;
  pending_steps_.clear();
  active_step_actions_.clear();
  state_.node_states.clear();
  state_.edge_states.clear();

  // Combine nodes + edges into a flat sorted view.
  struct Item
  {
    bool is_node;
    uint32_t seq;
    const void* ptr;
  };
  std::vector<Item> items;
  items.reserve(order.nodes.size() + order.edges.size());
  for (const auto& n : order.nodes) items.push_back({true, n.sequence_id, &n});
  for (const auto& e : order.edges) items.push_back({false, e.sequence_id, &e});
  std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
    return a.seq < b.seq;
  });

  for (const auto& it : items)
  {
    PendingStep ps;
    ps.is_node = it.is_node;
    ps.sequence_id = it.seq;
    bool released = false;
    if (it.is_node)
    {
      const auto& n = *static_cast<const vda5050_core::types::Node*>(it.ptr);
      ps.id = n.node_id;
      ps.end_pose = n.node_position;
      ps.actions = n.actions;
      released = n.released;
      NodeState ns;
      ns.node_id = n.node_id;
      ns.sequence_id = n.sequence_id;
      ns.node_position = n.node_position;
      ns.released = n.released;
      state_.node_states.push_back(ns);
    }
    else
    {
      const auto& e = *static_cast<const vda5050_core::types::Edge*>(it.ptr);
      ps.id = e.edge_id;
      ps.actions = e.actions;
      released = e.released;
      EdgeState es;
      es.edge_id = e.edge_id;
      es.sequence_id = e.sequence_id;
      es.released = e.released;
      state_.edge_states.push_back(es);
    }
    // Only released items past the AGV's current position are walkable.
    // Horizon items stay in state_.node_states/edge_states for the
    // master to see, but the mock won't drive them.
    if (released && it.seq > walked_through)
    {
      pending_steps_.push_back(std::move(ps));
    }
  }

  // Initialize ActionState entries for any node/edge actions.
  for (const auto& ps : pending_steps_)
  {
    for (const auto& a : ps.actions)
    {
      ActionState as;
      as.action_id = a.action_id;
      as.action_type = a.action_type;
      as.action_status = ActionStatus::WAITING;
      state_.action_states.push_back(as);
    }
  }

  VDA5050_INFO(
    "[mock_client] {} order id={} update_id={} ({} nodes / {} edges)",
    is_update ? "Updated" : "Received", order.order_id, order.order_update_id,
    order.nodes.size(), order.edges.size());
}

void MockClient::drain_instant_actions_locked()
{
  while (!pending_iactions_.empty())
  {
    const Action a = pending_iactions_.front();
    pending_iactions_.pop_front();

    // Find the matching ActionState we created in on_instant_actions.
    auto as_it = std::find_if(
      state_.action_states.begin(), state_.action_states.end(),
      [&](const ActionState& s) { return s.action_id == a.action_id; });

    auto set_status = [&](ActionStatus s, const char* desc = nullptr) {
      if (as_it == state_.action_states.end()) return;
      as_it->action_status = s;
      if (desc) as_it->result_description = desc;
    };

    // Mode-gate: the master's pre-send validators reject
    // mode-not-AUTOMATIC actions upstream — but if a request slips
    // through (or a test bypasses the master), the AGV must FAIL the
    // action with a clear reason. stateRequest / factsheetRequest are
    // exempt (operational).
    const bool exempt =
      (a.action_type == "stateRequest" || a.action_type == "factsheetRequest");
    if (!exempt && state_.operating_mode != OperatingMode::AUTOMATIC)
    {
      set_status(ActionStatus::FAILED, "mode_not_automatic");
      VDA5050_WARN(
        "[mock_client] action {} ({}) FAILED: not in AUTOMATIC", a.action_id,
        a.action_type);
      continue;
    }

    if (a.action_type == "stateRequest")
    {
      // Mock publishes State on every tick anyway, so the next tick's
      // State publish satisfies the request. Just ack FINISHED.
      set_status(ActionStatus::FINISHED);
      VDA5050_INFO("[mock_client] action {} (stateRequest) acked", a.action_id);
    }
    else if (a.action_type == "factsheetRequest")
    {
      // Defer the actual Factsheet publish to tick_loop, which runs
      // outside this lock (publish_factsheet_minimal would re-enter
      // ProtocolAdapter's mutex).
      factsheet_request_pending_ = true;
      set_status(ActionStatus::FINISHED);
      VDA5050_INFO(
        "[mock_client] action {} (factsheetRequest) acked", a.action_id);
    }
    else if (a.action_type == "cancelOrder")
    {
      // Drop active order + clear states; mark order_id="" so master
      // sees terminal phase.
      if (active_order_.has_value())
      {
        for (auto& s : state_.action_states)
        {
          if (s.action_status != ActionStatus::FINISHED)
            s.action_status = ActionStatus::FAILED;
        }
        active_order_.reset();
        state_.order_id = "";
        state_.order_update_id = 0;
        state_.node_states.clear();
        state_.edge_states.clear();
        state_.driving = false;
        pending_steps_.clear();
        active_step_actions_.clear();
      }
      set_status(ActionStatus::FINISHED);
      VDA5050_INFO("[mock_client] action {} (cancelOrder) acked", a.action_id);
    }
    else if (a.action_type == "startPause")
    {
      state_.paused = true;
      set_status(ActionStatus::FINISHED);
      VDA5050_INFO("[mock_client] action {} (startPause) acked", a.action_id);
    }
    else if (a.action_type == "stopPause")
    {
      state_.paused = false;
      set_status(ActionStatus::FINISHED);
      VDA5050_INFO("[mock_client] action {} (stopPause) acked", a.action_id);
    }
    else
    {
      // Custom action — pretend-run: WAITING (already) -> RUNNING (this
      // tick) -> FINISHED (we just complete it).
      set_status(ActionStatus::FINISHED);
      VDA5050_INFO(
        "[mock_client] action {} ({}) acked", a.action_id, a.action_type);
    }
  }
}

void MockClient::advance_step_locked()
{
  if (pending_steps_.empty() || state_.paused.value_or(false))
  {
    // No active step or paused — keep driving=false.
    if (pending_steps_.empty()) state_.driving = false;
    return;
  }

  PendingStep& step = pending_steps_.front();

  // Run any actions on this step. Simple model: 1 tick to complete
  // for NONE/SOFT, 2 ticks for HARD. Track via active_step_actions_
  // keyed by action_id.
  bool blocked_by_action = false;
  for (const auto& a : step.actions)
  {
    auto it = active_step_actions_.find(a.action_id);
    if (it == active_step_actions_.end())
    {
      // Not started yet. Set to RUNNING and seed countdown.
      int ticks = (a.blocking_type == BlockingType::HARD) ? 2 : 1;
      active_step_actions_[a.action_id] = ticks;
      for (auto& s : state_.action_states)
      {
        if (s.action_id == a.action_id)
        {
          s.action_status = ActionStatus::RUNNING;
          break;
        }
      }
      blocked_by_action = (a.blocking_type != BlockingType::NONE);
      continue;
    }
    if (it->second > 0)
    {
      --it->second;
      if (a.blocking_type != BlockingType::NONE) blocked_by_action = true;
      if (it->second == 0)
      {
        for (auto& s : state_.action_states)
        {
          if (s.action_id == a.action_id)
          {
            s.action_status = ActionStatus::FINISHED;
            break;
          }
        }
      }
    }
  }
  if (blocked_by_action)
  {
    state_.driving = false;
    return;
  }

  if (step.is_node)
  {
    // Reaching this node: update pose + last_node_*, pop NodeState,
    // pop pending_step.
    state_.last_node_id = step.id;
    state_.last_node_sequence_id = step.sequence_id;
    if (step.end_pose.has_value())
    {
      AGVPosition pose;
      pose.x = step.end_pose->x;
      pose.y = step.end_pose->y;
      pose.theta = step.end_pose->theta.value_or(0.0);
      pose.map_id = step.end_pose->map_id;
      pose.position_initialized = true;
      state_.agv_position = pose;
    }
    if (!state_.node_states.empty())
      state_.node_states.erase(state_.node_states.begin());
    state_.driving = false;
    pending_steps_.pop_front();
  }
  else
  {
    // Edge step: 1 tick traversal. driving=true while traversing; pose
    // doesn't change here (the next node step will set it). Pop the
    // EdgeState immediately on the tick we "start" the edge — works
    // for this simple mock where one tick = one edge.
    state_.driving = true;
    if (!state_.edge_states.empty())
      state_.edge_states.erase(state_.edge_states.begin());
    pending_steps_.pop_front();
  }
}

// --- tick loop --------------------------------------------------------------

void MockClient::tick_loop()
{
  while (g_alive)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(cli_.tick_ms));
    if (!g_alive) break;

    // Apply pending signal-driven flips before progressing the state
    // machine so this tick reflects the new mode/pause already.
    if (g_pending_mode_flip.exchange(false))
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      state_.operating_mode =
        (state_.operating_mode == OperatingMode::AUTOMATIC)
          ? OperatingMode::MANUAL
          : OperatingMode::AUTOMATIC;
      VDA5050_INFO(
        "[mock_client] mode flipped -> {}", mode_label(state_.operating_mode));
    }
    if (g_pending_pause_flip.exchange(false))
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      state_.paused = !state_.paused.value_or(false);
      VDA5050_INFO(
        "[mock_client] paused flipped -> {}",
        state_.paused.value_or(false) ? "true" : "false");
    }

    bool republish_factsheet = false;
    State snapshot;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      drain_instant_actions_locked();
      republish_factsheet = factsheet_request_pending_;
      factsheet_request_pending_ = false;
      advance_step_locked();
      snapshot = state_;  // copy under the lock
    }

    // Heartbeat the Connection ONLINE on every tick. VDA5050 §6.14
    // requires a 15s connection heartbeat; here we send it faster so
    // that a master started in the middle of a mock run picks up the
    // connection state on its first subscribe, without relying on
    // broker retain delivery (which has been observed to race on first
    // subscribe in some broker configurations).
    publish_connection(ConnectionState::ONLINE);

    if (republish_factsheet)
    {
      publish_factsheet_minimal();
    }

    publish_state_snapshot(snapshot);
  }

  // Graceful OFFLINE. publish_connection is safe to call here — we
  // hold no locks at this point.
  publish_connection(ConnectionState::OFFLINE);
  VDA5050_INFO(
    "[mock_client] published Connection{{OFFLINE}}, exiting tick loop");
}

}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGUSR1, on_signal);
  std::signal(SIGUSR2, on_signal);

  Cli cli = parse_cli(argc, argv);
  try
  {
    MockClient mock(cli);
    mock.run();
  }
  catch (const std::exception& e)
  {
    fmt::print(stderr, "mock_client fatal: {}\n", e.what());
    return 1;
  }
  return 0;
}
