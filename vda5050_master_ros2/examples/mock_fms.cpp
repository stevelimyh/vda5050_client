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

// mock_fms — FMS-side reference exec for interop testing. Drives the
// master's 10 ROS 2 service endpoints and subscribes to the per-AGV
// state / connection / order_status topics.
//
// Three modes, mutually exclusive (default = interactive):
//   --scenario <name>   Run a preset scripted scenario; exit non-zero on
//                       failure. Used for automated runbook walks.
//                       Names: happy-path | stitch | broker-bounce |
//                       onboard-flood.
//   --interactive       REPL prompt; one command per line dispatches one
//                       service call with pretty-printed output. The
//                       "FMS console" for ad-hoc operator-side work.
//   --monitor <serial>  Subscribe to that AGV's state / connection /
//                       order_status topics; pretty-print only on change.
//                       Ctrl-C to exit.
//
// Run examples:
//   ros2 run vda5050_master_ros2 mock_fms --scenario happy-path
//   ros2 run vda5050_master_ros2 mock_fms --interactive
//   ros2 run vda5050_master_ros2 mock_fms --monitor S001

#include <fmt/core.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "vda5050_master_ros2/msg/agv_key.hpp"
#include "vda5050_master_ros2/msg/agv_onboard_spec.hpp"
#include "vda5050_master_ros2/srv/assign_instant_actions.hpp"
#include "vda5050_master_ros2/srv/assign_order.hpp"
#include "vda5050_master_ros2/srv/discard_mode_cancelled_queue.hpp"
#include "vda5050_master_ros2/srv/get_device_status.hpp"
#include "vda5050_master_ros2/srv/get_loaded_map.hpp"
#include "vda5050_master_ros2/srv/get_master_broker_status.hpp"
#include "vda5050_master_ros2/srv/get_order_status.hpp"
#include "vda5050_master_ros2/srv/offboard_agv.hpp"
#include "vda5050_master_ros2/srv/offboard_agv_batch.hpp"
#include "vda5050_master_ros2/srv/onboard_agv.hpp"
#include "vda5050_master_ros2/srv/onboard_agv_batch.hpp"
#include "vda5050_master_ros2/srv/resume_mode_cancelled_queue.hpp"

#include "vda5050_interfaces/msg/connection.hpp"
#include "vda5050_interfaces/msg/instant_actions.hpp"
#include "vda5050_interfaces/msg/order.hpp"
#include "vda5050_interfaces/msg/state.hpp"
#include "vda5050_master_ros2/msg/order_status.hpp"

namespace {

// ============================================================================
// Constants
// ============================================================================

constexpr int kDefaultTimeoutMs = 5000;
constexpr const char* kDefaultMaster = "vda5050_master";
constexpr const char* kDefaultMfg = "Manufacturer";
constexpr const char* kDefaultSerial = "S001";

std::atomic_bool g_running{true};
void signal_handler(int sig)
{
  (void)sig;
  g_running = false;
}

// ============================================================================
// Service-type aliases (cut the verbose namespace at every call-site)
// ============================================================================

using AssignOrder = vda5050_master_ros2::srv::AssignOrder;
using AssignInstantActions = vda5050_master_ros2::srv::AssignInstantActions;
using OnboardAGV = vda5050_master_ros2::srv::OnboardAGV;
using OnboardAGVBatch = vda5050_master_ros2::srv::OnboardAGVBatch;
using OffboardAGV = vda5050_master_ros2::srv::OffboardAGV;
using OffboardAGVBatch = vda5050_master_ros2::srv::OffboardAGVBatch;
using GetDeviceStatus = vda5050_master_ros2::srv::GetDeviceStatus;
using GetOrderStatus = vda5050_master_ros2::srv::GetOrderStatus;
using GetLoadedMap = vda5050_master_ros2::srv::GetLoadedMap;
using GetMasterBrokerStatus = vda5050_master_ros2::srv::GetMasterBrokerStatus;
using ResumeQueue = vda5050_master_ros2::srv::ResumeModeCancelledQueue;
using DiscardQueue = vda5050_master_ros2::srv::DiscardModeCancelledQueue;

// ============================================================================
// FmsContext — shared state across modes
// ============================================================================

struct FmsContext
{
  rclcpp::Node::SharedPtr node;
  std::string mfg = kDefaultMfg;
  std::string serial = kDefaultSerial;
  std::string master_ns = kDefaultMaster;
  std::chrono::milliseconds timeout{kDefaultTimeoutMs};
};

// ============================================================================
// Service-call helpers
// ============================================================================

// Discovery timeout is separate from the per-call timeout. On a freshly
// spawned mock_fms (cold DDS cache), wait_for_service can legitimately
// take several seconds before the master's service shows up — that's
// service discovery, not service slowness. Reusing the same `timeout`
// for both meant a slow discovery on the first call left no budget for
// the actual round-trip, producing a spurious "call timed out". The
// scenario-level pass/fail wants to flag *real* unresponsive services,
// not normal discovery latency, so we give discovery its own generous
// budget and reserve the caller's timeout for the round-trip.
constexpr std::chrono::seconds kServiceDiscoveryTimeout{30};

template <typename SrvT>
std::shared_ptr<typename SrvT::Response> call_service(
  rclcpp::Node::SharedPtr node, const std::string& name,
  typename SrvT::Request::SharedPtr req, std::chrono::milliseconds timeout)
{
  auto client = node->create_client<SrvT>(name);
  if (!client->wait_for_service(kServiceDiscoveryTimeout))
  {
    fmt::print(stderr, "[mock_fms] service '{}' not available\n", name);
    return nullptr;
  }
  auto future = client->async_send_request(req);
  if (
    rclcpp::spin_until_future_complete(node, future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    fmt::print(stderr, "[mock_fms] service '{}' call timed out\n", name);
    return nullptr;
  }
  return future.get();
}

// Helpers for the scripted-scenario PASS/FAIL output. Inline functions
// (not macros) avoid cpplint's readability/braces tripping on the
// expanded if-body that the project's clang-format formats Allman-style.
// They return the truth/failure so callers do `if (!ok(...)) return false;`.
template <typename T>
[[nodiscard]]
bool ok_nn(const T& v, const std::string& msg)
{
  if (v) return true;
  fmt::print(stderr, "  FAIL: {}\n", msg);
  return false;
}
template <typename A, typename E>
[[nodiscard]]
bool ok_eq(const A& a, const E& e, const std::string& msg)
{
  if (a == e) return true;
  fmt::print(stderr, "  FAIL ({}): expected {} got {}\n", msg, e, a);
  return false;
}

// ============================================================================
// Order builder — used by both scripted scenarios and interactive `order`
// ============================================================================

vda5050_interfaces::msg::Order make_two_node_order(
  const std::string& mfg, const std::string& serial,
  const std::string& order_id, uint32_t order_update_id)
{
  vda5050_interfaces::msg::Order order;
  order.header.header_id = 0;
  order.header.timestamp = 1778746000000ULL;
  order.header.version = "2.0.0";
  order.header.manufacturer = mfg;
  order.header.serial_number = serial;
  order.order_id = order_id;
  order.order_update_id = order_update_id;

  auto make_node =
    [](const std::string& id, uint32_t seq, double x, double y, bool released) {
      vda5050_interfaces::msg::Node n;
      n.node_id = id;
      n.sequence_id = seq;
      n.released = released;
      vda5050_interfaces::msg::NodePosition p;
      p.x = x;
      p.y = y;
      p.theta = {0.0};
      p.allowed_deviation_x_y = {0.5};
      p.allowed_deviation_theta = {0.1};
      p.map_id = "warehouse_floor1";
      n.node_position.push_back(p);
      return n;
    };
  auto make_edge = [](
                     const std::string& id, uint32_t seq,
                     const std::string& from, const std::string& to,
                     bool released) {
    vda5050_interfaces::msg::Edge e;
    e.edge_id = id;
    e.sequence_id = seq;
    e.start_node_id = from;
    e.end_node_id = to;
    e.released = released;
    e.max_speed = {1.0};
    e.length = {5.0};
    return e;
  };
  order.nodes.push_back(make_node("N0", 0, 0.0, 0.0, true));
  order.edges.push_back(make_edge("E1", 1, "N0", "N1", true));
  order.nodes.push_back(make_node("N1", 2, 5.0, 0.0, true));
  return order;
}

// Adds an unreleased horizon node N2 to an already-built order (for stitch).
void append_horizon_n2(vda5050_interfaces::msg::Order& order)
{
  vda5050_interfaces::msg::Node n;
  n.node_id = "N2";
  n.sequence_id = 4;
  n.released = false;
  vda5050_interfaces::msg::NodePosition p;
  p.x = 5.0;
  p.y = 5.0;
  p.theta = {0.0};
  p.allowed_deviation_x_y = {0.5};
  p.allowed_deviation_theta = {0.1};
  p.map_id = "warehouse_floor1";
  n.node_position.push_back(p);
  order.nodes.push_back(n);
  vda5050_interfaces::msg::Edge e;
  e.edge_id = "E2";
  e.sequence_id = 3;
  e.start_node_id = "N1";
  e.end_node_id = "N2";
  e.released = false;
  e.max_speed = {1.0};
  e.length = {5.0};
  order.edges.push_back(e);
}

// ============================================================================
// Wait for state-topic predicate (used by scripted scenarios)
// ============================================================================

template <typename Predicate>
bool wait_for_order_status(
  FmsContext& ctx, Predicate pred, std::chrono::milliseconds timeout)
{
  const auto topic =
    fmt::format("/{}/{}/{}/order_status", ctx.master_ns, ctx.mfg, ctx.serial);
  std::atomic_bool matched{false};
  auto sub =
    ctx.node->create_subscription<vda5050_master_ros2::msg::OrderStatus>(
      topic, 10, [&](vda5050_master_ros2::msg::OrderStatus::SharedPtr msg) {
        if (pred(*msg)) matched = true;
      });
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!matched && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(ctx.node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return matched;
}

// ============================================================================
// Scenario hygiene helpers — addresses critic concern C3 (cross-scenario
// residual state false-passes). Each scenario MUST use a unique order_id
// so wait_for_order_status predicates don't match stale topic traffic
// from a prior scenario, and SHOULD offboard the AGV on exit so the next
// scenario starts from a clean master-side view.
// ============================================================================

std::string unique_order_id(const std::string& prefix)
{
  // epoch-ms + pid + counter — collision-free across mock_fms restarts
  // and across scenarios within a single run.
  static std::atomic<uint32_t> counter{0};
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  return fmt::format(
    "{}-{}-{}-{}", prefix, ms, getpid(),
    counter.fetch_add(1, std::memory_order_relaxed));
}

// RAII guard: best-effort offboard of (mfg, serial) when the scenario
// function exits — pass or fail. Idempotent if the AGV wasn't onboarded.
class OffboardOnExit
{
public:
  OffboardOnExit(FmsContext& ctx, std::string mfg, std::string serial)
  : ctx_(ctx), mfg_(std::move(mfg)), serial_(std::move(serial))
  {
  }
  ~OffboardOnExit()
  {
    auto req = std::make_shared<OffboardAGV::Request>();
    req->agv.manufacturer = mfg_;
    req->agv.serial_number = serial_;
    // Short timeout; if it fails, don't throw from the destructor.
    try
    {
      (void)call_service<OffboardAGV>(
        ctx_.node, fmt::format("/{}/offboard_agv", ctx_.master_ns), req,
        std::chrono::milliseconds(1500));
    }
    catch (...)
    {
    }
  }

private:
  FmsContext& ctx_;
  std::string mfg_;
  std::string serial_;
};

// ============================================================================
// Mode 1 — Scripted scenarios
// ============================================================================

bool scenario_happy_path(FmsContext& ctx)
{
  fmt::print(
    "→ scenario_happy_path (mfg={}, serial={})\n", ctx.mfg, ctx.serial);

  // Best-effort offboard on scenario exit so the next scenario starts
  // with a clean master-side AGV state. RAII guarantees offboard runs
  // even on early-return failure paths.
  OffboardOnExit offboard_guard(ctx, ctx.mfg, ctx.serial);

  // Unique order_id per run — ensures wait_for_order_status predicates
  // can't match residual topic traffic from a prior scenario.
  const std::string order_id = unique_order_id("happy");

  // 1. Onboard (single-AGV)
  {
    auto req = std::make_shared<OnboardAGV::Request>();
    req->agv.manufacturer = ctx.mfg;
    req->agv.serial_number = ctx.serial;
    auto resp = call_service<OnboardAGV>(
      ctx.node, fmt::format("/{}/onboard_agv", ctx.master_ns), req,
      ctx.timeout);
    if (!ok_nn(resp, "onboard_agv response")) return false;
    // Accept SUCCESS (newly onboarded) and ALREADY_ONBOARDED — both
    // leave the master ready to receive orders for this AGV.
    if (
      resp->status != OnboardAGV::Response::SUCCESS &&
      resp->status != OnboardAGV::Response::ALREADY_ONBOARDED)
    {
      fmt::print(
        stderr, "  FAIL (onboard): status={}\n",
        static_cast<int>(resp->status));
      return false;
    }
    fmt::print("  onboard status={}\n", static_cast<int>(resp->status));
  }

  // 2. Wait for state to arrive (AGV publishes at 1 Hz; ~3s is plenty)
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    bool seen = false;
    while (std::chrono::steady_clock::now() < deadline && !seen)
    {
      auto req = std::make_shared<GetDeviceStatus::Request>();
      req->manufacturer = ctx.mfg;
      req->serial_number = ctx.serial;
      auto resp = call_service<GetDeviceStatus>(
        ctx.node, fmt::format("/{}/get_device_status", ctx.master_ns), req,
        ctx.timeout);
      if (resp && !resp->state.empty() && !resp->connection.empty())
      {
        seen = true;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    if (!ok_nn(seen, "state + connection received")) return false;
    fmt::print("  state + connection arrived\n");
  }

  // 3. AssignOrder
  {
    auto req = std::make_shared<AssignOrder::Request>();
    req->manufacturer = ctx.mfg;
    req->serial_number = ctx.serial;
    req->order = make_two_node_order(ctx.mfg, ctx.serial, order_id, 0);
    auto resp = call_service<AssignOrder>(
      ctx.node, fmt::format("/{}/assign_order", ctx.master_ns), req,
      ctx.timeout);
    if (!ok_nn(resp, "assign_order response")) return false;
    if (!ok_eq(
          static_cast<int>(resp->decision),
          static_cast<int>(AssignOrder::Response::ASSIGNED), "assign decision"))
      return false;
    fmt::print("  order ASSIGNED (order_id={})\n", order_id);
  }

  // 4. Wait for OrderStatus to reach last_node_id=N1 — assert BOTH
  // order_id AND order_update_id match what we just sent, so stale
  // residual topic traffic from a prior scenario can't false-match.
  {
    bool ok = wait_for_order_status(
      ctx,
      [&order_id](const vda5050_master_ros2::msg::OrderStatus& s) {
        return s.order_id == order_id && s.order_update_id == 0 &&
               s.last_node_id == "N1";
      },
      std::chrono::seconds(15));
    if (!ok_nn(ok, "order_status last_node_id=N1 within 15s")) return false;
    fmt::print("  order reached N1 (last_node_id=N1)\n");
  }

  return true;
}

bool scenario_stitch(FmsContext& ctx)
{
  fmt::print("→ scenario_stitch (mfg={}, serial={})\n", ctx.mfg, ctx.serial);

  // C3 hygiene: offboard on exit + unique order_id per run.
  OffboardOnExit offboard_guard(ctx, ctx.mfg, ctx.serial);
  const std::string order_id = unique_order_id("stitch");

  // Onboard + wait for state (re-use happy logic via inline calls).
  {
    auto req = std::make_shared<OnboardAGV::Request>();
    req->agv.manufacturer = ctx.mfg;
    req->agv.serial_number = ctx.serial;
    auto resp = call_service<OnboardAGV>(
      ctx.node, fmt::format("/{}/onboard_agv", ctx.master_ns), req,
      ctx.timeout);
    if (!ok_nn(resp, "onboard_agv response")) return false;
    if (
      resp->status != OnboardAGV::Response::SUCCESS &&
      resp->status != OnboardAGV::Response::ALREADY_ONBOARDED)
      return false;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    fmt::print("  onboard + state ready\n");
  }

  // Send base order with horizon N2.
  {
    auto req = std::make_shared<AssignOrder::Request>();
    req->manufacturer = ctx.mfg;
    req->serial_number = ctx.serial;
    auto order = make_two_node_order(ctx.mfg, ctx.serial, order_id, 0);
    append_horizon_n2(order);
    req->order = order;
    auto resp = call_service<AssignOrder>(
      ctx.node, fmt::format("/{}/assign_order", ctx.master_ns), req,
      ctx.timeout);
    if (!ok_nn(resp, "stitch base assign_order response")) return false;
    if (!ok_eq(
          static_cast<int>(resp->decision),
          static_cast<int>(AssignOrder::Response::ASSIGNED),
          "stitch base assign"))
      return false;
    fmt::print("  base + horizon order ASSIGNED (order_id={})\n", order_id);
  }

  // Wait for AGV to reach N1 (last released base node). Assert
  // order_update_id==0 so this can't match the post-update topic
  // traffic that will follow.
  {
    bool ok = wait_for_order_status(
      ctx,
      [&order_id](const vda5050_master_ros2::msg::OrderStatus& s) {
        return s.order_id == order_id && s.order_update_id == 0 &&
               s.last_node_id == "N1";
      },
      std::chrono::seconds(15));
    if (!ok_nn(ok, "stitch base reaches N1 within 15s")) return false;
    fmt::print("  base reached N1, horizon still parked at N2\n");
  }

  // Now send the update: same order_id, order_update_id=1. Per VDA5050
  // §6.6.2, an update must NOT alter the released base — so the update
  // contains ONLY the stitch point (N1, identical to its prior shape)
  // plus the newly-released horizon (N2). Master's stitcher merges this
  // with the active base.
  {
    auto req = std::make_shared<AssignOrder::Request>();
    req->manufacturer = ctx.mfg;
    req->serial_number = ctx.serial;
    vda5050_interfaces::msg::Order order;
    order.header.header_id = 0;
    order.header.timestamp = 1778746000000ULL;
    order.header.version = "2.0.0";
    order.header.manufacturer = ctx.mfg;
    order.header.serial_number = ctx.serial;
    order.order_id = order_id;
    order.order_update_id = 1;
    // Stitch point: N1, sequence_id=2 (same as in the base).
    vda5050_interfaces::msg::Node n1;
    n1.node_id = "N1";
    n1.sequence_id = 2;
    n1.released = true;
    vda5050_interfaces::msg::NodePosition p1;
    p1.x = 5.0;
    p1.y = 0.0;
    p1.theta = {0.0};
    p1.allowed_deviation_x_y = {0.5};
    p1.allowed_deviation_theta = {0.1};
    p1.map_id = "warehouse_floor1";
    n1.node_position.push_back(p1);
    order.nodes.push_back(n1);
    // Edge E2 (newly released): N1 → N2, sequence_id=3.
    vda5050_interfaces::msg::Edge e2;
    e2.edge_id = "E2";
    e2.sequence_id = 3;
    e2.start_node_id = "N1";
    e2.end_node_id = "N2";
    e2.released = true;
    e2.max_speed = {1.0};
    e2.length = {5.0};
    order.edges.push_back(e2);
    // Newly released horizon: N2, sequence_id=4.
    vda5050_interfaces::msg::Node n2;
    n2.node_id = "N2";
    n2.sequence_id = 4;
    n2.released = true;
    vda5050_interfaces::msg::NodePosition p2;
    p2.x = 5.0;
    p2.y = 5.0;
    p2.theta = {0.0};
    p2.allowed_deviation_x_y = {0.5};
    p2.allowed_deviation_theta = {0.1};
    p2.map_id = "warehouse_floor1";
    n2.node_position.push_back(p2);
    order.nodes.push_back(n2);
    req->order = order;
    auto resp = call_service<AssignOrder>(
      ctx.node, fmt::format("/{}/assign_order", ctx.master_ns), req,
      ctx.timeout);
    if (!ok_nn(resp, "stitch update assign_order response")) return false;
    // Either ASSIGNED (immediate stitch) or STITCH_QUEUED (waiting for
    // state to land on stitch point) is acceptable.
    auto d = static_cast<int>(resp->decision);
    bool decision_ok = d == AssignOrder::Response::ASSIGNED ||
                       d == AssignOrder::Response::STITCH_QUEUED;
    if (!ok_nn(
          decision_ok,
          fmt::format("stitch update decision unexpectedly = {}", d)))
      return false;
    fmt::print(
      "  horizon extension {} (decision={})\n",
      d == AssignOrder::Response::ASSIGNED ? "ASSIGNED" : "QUEUED", d);
  }

  // Wait for AGV to reach N2 — must also see order_update_id==1 so we
  // know the master actually published the update and the AGV ack'd
  // the post-stitch portion, not just the residual base.
  {
    bool ok = wait_for_order_status(
      ctx,
      [&order_id](const vda5050_master_ros2::msg::OrderStatus& s) {
        return s.order_id == order_id && s.order_update_id == 1 &&
               s.last_node_id == "N2";
      },
      std::chrono::seconds(20));
    if (!ok_nn(ok, "stitch reaches N2 within 20s")) return false;
    fmt::print("  AGV reached extended horizon N2\n");
  }

  return true;
}

bool scenario_broker_bounce(FmsContext& ctx)
{
  fmt::print(
    "→ scenario_broker_bounce (mfg={}, serial={})\n", ctx.mfg, ctx.serial);
  // Snapshot the broker status; this scenario does NOT physically bounce
  // the broker — that's an operator action. We assert the service responds
  // and that connected=true at the start. The operator runs the runbook
  // step `sudo systemctl restart mosquitto` between two invocations.
  auto req = std::make_shared<GetMasterBrokerStatus::Request>();
  auto resp = call_service<GetMasterBrokerStatus>(
    ctx.node, fmt::format("/{}/get_master_broker_status", ctx.master_ns), req,
    ctx.timeout);
  if (!ok_nn(resp, "get_master_broker_status response")) return false;
  if (!ok_eq(static_cast<int>(resp->connected), 1, "broker connected at start"))
    return false;
  fmt::print(
    "  broker status reports connected (reconnect_count={})\n",
    resp->reconnect_count);
  fmt::print(
    "  (operator: run `sudo systemctl restart mosquitto` and re-run "
    "this scenario; expect reconnect_count to increment)\n");
  return true;
}

bool scenario_onboard_flood(FmsContext& ctx)
{
  fmt::print("→ scenario_onboard_flood\n");
  const std::vector<std::string> serials = {"FLOOD01", "FLOOD02", "FLOOD03"};

  // Batch onboard via OnboardAGVBatch — one round-trip for N AGVs.
  {
    auto req = std::make_shared<OnboardAGVBatch::Request>();
    for (const auto& s : serials)
    {
      vda5050_master_ros2::msg::AGVOnboardSpec entry;
      entry.manufacturer = ctx.mfg;
      entry.serial_number = s;
      req->agvs.push_back(entry);
    }
    auto resp = call_service<OnboardAGVBatch>(
      ctx.node, fmt::format("/{}/onboard_agv_batch", ctx.master_ns), req,
      ctx.timeout);
    if (!ok_nn(resp, "onboard_agv_batch")) return false;
    if (!resp->failed.empty()) return false;
    fmt::print(
      "  onboarded {} of {} (others were already onboarded)\n",
      resp->onboarded.size(), serials.size());
  }

  // Cleanup: batch offboard.
  {
    auto req = std::make_shared<OffboardAGVBatch::Request>();
    for (const auto& s : serials)
    {
      vda5050_master_ros2::msg::AGVKey key;
      key.manufacturer = ctx.mfg;
      key.serial_number = s;
      req->agvs.push_back(key);
    }
    auto resp = call_service<OffboardAGVBatch>(
      ctx.node, fmt::format("/{}/offboard_agv_batch", ctx.master_ns), req,
      ctx.timeout);
    if (!ok_nn(resp, "offboard_agv_batch")) return false;
    fmt::print("  offboarded {}\n", resp->offboarded_count);
  }
  fmt::print("  flood completed and cleaned up\n");
  return true;
}

int run_scripted(FmsContext& ctx, const std::string& name)
{
  bool ok = false;
  if (name == "happy-path")
    ok = scenario_happy_path(ctx);
  else if (name == "stitch")
    ok = scenario_stitch(ctx);
  else if (name == "broker-bounce")
    ok = scenario_broker_bounce(ctx);
  else if (name == "onboard-flood")
    ok = scenario_onboard_flood(ctx);
  else
  {
    fmt::print(stderr, "unknown scenario '{}'\n", name);
    return 2;
  }
  fmt::print("{} {}\n", ok ? "PASS:" : "FAIL:", name);
  return ok ? 0 : 1;
}

// ============================================================================
// Mode 3 — Monitor
// ============================================================================

class TopicMonitor
{
public:
  TopicMonitor(
    rclcpp::Node::SharedPtr node, const std::string& ns, const std::string& mfg,
    const std::string& serial)
  : node_(node)
  {
    using std::placeholders::_1;
    state_sub_ = node_->create_subscription<vda5050_interfaces::msg::State>(
      fmt::format("/{}/{}/{}/state", ns, mfg, serial), 10,
      std::bind(&TopicMonitor::on_state, this, _1));
    conn_sub_ = node_->create_subscription<vda5050_interfaces::msg::Connection>(
      fmt::format("/{}/{}/{}/connection", ns, mfg, serial), 10,
      std::bind(&TopicMonitor::on_conn, this, _1));
    order_sub_ =
      node_->create_subscription<vda5050_master_ros2::msg::OrderStatus>(
        fmt::format("/{}/{}/{}/order_status", ns, mfg, serial), 10,
        std::bind(&TopicMonitor::on_order, this, _1));
    fmt::print(
      "[mock_fms] monitoring {}/{}/{}; Ctrl-C to exit\n", ns, mfg, serial);
  }

private:
  static std::string ts()
  {
    auto t = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(t);
    char buf[16];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&tt));
    return buf;
  }
  void on_state(vda5050_interfaces::msg::State::SharedPtr m)
  {
    std::string node = m->last_node_id;
    std::string ord = m->order_id;
    std::string mode = m->operating_mode;
    bool driving = m->driving;
    bool changed = ord != last_order_ || node != last_node_ ||
                   mode != last_mode_ || driving != last_driving_;
    if (!changed) return;
    fmt::print(
      "[{}] STATE  order={}/{} node={} driving={} mode={}\n", ts(), ord,
      m->order_update_id, node.empty() ? "-" : node, driving, mode);
    last_order_ = ord;
    last_node_ = node;
    last_mode_ = mode;
    last_driving_ = driving;
  }
  void on_conn(vda5050_interfaces::msg::Connection::SharedPtr m)
  {
    if (m->connection_state == last_conn_) return;
    fmt::print("[{}] CONN   {}\n", ts(), m->connection_state);
    last_conn_ = m->connection_state;
  }
  void on_order(vda5050_master_ros2::msg::OrderStatus::SharedPtr m)
  {
    if (m->phase == last_phase_ && m->last_node_id == last_oss_node_) return;
    fmt::print(
      "[{}] ORDER  phase={} last_node={} base/horizon={}/{}\n", ts(),
      static_cast<int>(m->phase),
      m->last_node_id.empty() ? "-" : m->last_node_id, m->nodes_in_base,
      m->nodes_in_horizon);
    last_phase_ = m->phase;
    last_oss_node_ = m->last_node_id;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vda5050_interfaces::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<vda5050_interfaces::msg::Connection>::SharedPtr
    conn_sub_;
  rclcpp::Subscription<vda5050_master_ros2::msg::OrderStatus>::SharedPtr
    order_sub_;
  std::string last_order_, last_node_, last_mode_, last_conn_, last_oss_node_;
  bool last_driving_ = false;
  uint8_t last_phase_ = 255;
};

int run_monitor(FmsContext& ctx)
{
  TopicMonitor mon(ctx.node, ctx.master_ns, ctx.mfg, ctx.serial);
  while (g_running && rclcpp::ok())
  {
    rclcpp::spin_some(ctx.node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}

// ============================================================================
// Mode 2 — Interactive REPL
// ============================================================================

std::vector<std::string> split_ws(const std::string& line)
{
  std::vector<std::string> out;
  std::istringstream iss(line);
  std::string tok;
  while (iss >> tok) out.push_back(tok);
  return out;
}

void print_help()
{
  fmt::print(R"(commands:
  help                                Show this list
  target <mfg> <serial>               Set default AGV for following commands
  onboard [mfg] [serial]              Onboard the AGV
  offboard [mfg] [serial]             Offboard the AGV
  status [serial]                     Get device status (state + connection)
  order-status [serial]               Get order status
  map                                 Get loaded map summary
  broker                              Get master broker status
  order [serial]                      Send a two-node demo order (N0→N1)
  iaction [serial] <action_type>      Send a one-shot instant action
  resume [serial]                     Resume mode-cancelled queue
  discard [serial]                    Discard mode-cancelled queue
  monitor [serial]                    Live state/connection/order_status feed
  quit | exit                         Leave the REPL
)");
}

int run_repl(FmsContext& ctx)
{
  fmt::print(
    "mock_fms interactive — master_ns=/{}  default AGV={}/{}\n"
    "type `help` for commands.\n",
    ctx.master_ns, ctx.mfg, ctx.serial);

  std::string line;
  while (g_running && rclcpp::ok())
  {
    fmt::print("fms> ");
    std::cout.flush();
    if (!std::getline(std::cin, line)) break;
    auto tokens = split_ws(line);
    if (tokens.empty()) continue;
    const auto& cmd = tokens[0];
    auto arg_serial = [&](size_t pos) -> std::string {
      return tokens.size() > pos ? tokens[pos] : ctx.serial;
    };

    if (cmd == "help")
    {
      print_help();
      continue;
    }
    if (cmd == "quit" || cmd == "exit") break;

    if (cmd == "target" && tokens.size() >= 3)
    {
      ctx.mfg = tokens[1];
      ctx.serial = tokens[2];
      fmt::print("  target set: {}/{}\n", ctx.mfg, ctx.serial);
      continue;
    }
    if (cmd == "onboard")
    {
      auto mfg = tokens.size() > 1 ? tokens[1] : ctx.mfg;
      auto serial = tokens.size() > 2 ? tokens[2] : ctx.serial;
      auto req = std::make_shared<OnboardAGV::Request>();
      req->agv.manufacturer = mfg;
      req->agv.serial_number = serial;
      auto r = call_service<OnboardAGV>(
        ctx.node, fmt::format("/{}/onboard_agv", ctx.master_ns), req,
        ctx.timeout);
      if (r) fmt::print("  onboard status={}\n", static_cast<int>(r->status));
      continue;
    }
    if (cmd == "offboard")
    {
      auto mfg = tokens.size() > 1 ? tokens[1] : ctx.mfg;
      auto serial = tokens.size() > 2 ? tokens[2] : ctx.serial;
      auto req = std::make_shared<OffboardAGV::Request>();
      req->agv.manufacturer = mfg;
      req->agv.serial_number = serial;
      auto r = call_service<OffboardAGV>(
        ctx.node, fmt::format("/{}/offboard_agv", ctx.master_ns), req,
        ctx.timeout);
      if (r) fmt::print("  offboard status={}\n", static_cast<int>(r->status));
      continue;
    }
    if (cmd == "status")
    {
      auto req = std::make_shared<GetDeviceStatus::Request>();
      req->manufacturer = ctx.mfg;
      req->serial_number = arg_serial(1);
      auto r = call_service<GetDeviceStatus>(
        ctx.node, fmt::format("/{}/get_device_status", ctx.master_ns), req,
        ctx.timeout);
      if (r)
      {
        const bool has_state = !r->state.empty();
        const bool has_connection = !r->connection.empty();
        const bool has_factsheet = !r->factsheet.empty();
        fmt::print(
          "  has_state={} has_connection={} has_factsheet={}\n", has_state,
          has_connection, has_factsheet);
        if (has_state)
        {
          const auto& s = r->state[0];
          fmt::print(
            "  state: order_id={} order_update_id={} "
            "last_node={} mode={} driving={}\n",
            s.order_id, s.order_update_id,
            s.last_node_id.empty() ? "-" : s.last_node_id, s.operating_mode,
            s.driving);
        }
        if (has_connection)
        {
          fmt::print("  conn: {}\n", r->connection[0].connection_state);
        }
      }
      continue;
    }
    if (cmd == "order-status")
    {
      auto req = std::make_shared<GetOrderStatus::Request>();
      req->manufacturer = ctx.mfg;
      req->serial_number = arg_serial(1);
      auto r = call_service<GetOrderStatus>(
        ctx.node, fmt::format("/{}/get_order_status", ctx.master_ns), req,
        ctx.timeout);
      if (r)
      {
        const auto& s = r->order_status;
        fmt::print(
          "  order_id={} order_update_id={} phase={} "
          "last_node={} base/horizon={}/{}\n",
          s.order_id, s.order_update_id, static_cast<int>(s.phase),
          s.last_node_id.empty() ? "-" : s.last_node_id, s.nodes_in_base,
          s.nodes_in_horizon);
      }
      continue;
    }
    if (cmd == "map")
    {
      auto req = std::make_shared<GetLoadedMap::Request>();
      auto r = call_service<GetLoadedMap>(
        ctx.node, fmt::format("/{}/get_loaded_map", ctx.master_ns), req,
        ctx.timeout);
      if (r)
      {
        fmt::print(
          "  status={} map_id={} version={} ({} nodes, {} edges)\n",
          static_cast<int>(r->status), r->map_id, r->map_version, r->node_count,
          r->edge_count);
      }
      continue;
    }
    if (cmd == "broker")
    {
      auto req = std::make_shared<GetMasterBrokerStatus::Request>();
      auto r = call_service<GetMasterBrokerStatus>(
        ctx.node, fmt::format("/{}/get_master_broker_status", ctx.master_ns),
        req, ctx.timeout);
      if (r)
      {
        if (r->last_disconnect_at.empty())
        {
          fmt::print(
            "  connected={} reconnect_count={} last_disconnect_at=-\n",
            static_cast<int>(r->connected), r->reconnect_count);
        }
        else
        {
          fmt::print(
            "  connected={} reconnect_count={} last_disconnect_at={}.{}\n",
            static_cast<int>(r->connected), r->reconnect_count,
            r->last_disconnect_at[0].sec, r->last_disconnect_at[0].nanosec);
        }
      }
      continue;
    }
    if (cmd == "order")
    {
      auto req = std::make_shared<AssignOrder::Request>();
      req->manufacturer = ctx.mfg;
      req->serial_number = arg_serial(1);
      static uint32_t order_seq = 0;
      req->order = make_two_node_order(
        ctx.mfg, req->serial_number, fmt::format("repl-{}", order_seq++), 0);
      auto r = call_service<AssignOrder>(
        ctx.node, fmt::format("/{}/assign_order", ctx.master_ns), req,
        ctx.timeout);
      if (r)
      {
        fmt::print(
          "  decision={} order_id={}\n", static_cast<int>(r->decision),
          r->order_id);
      }
      continue;
    }
    if (cmd == "iaction" && tokens.size() >= 2)
    {
      auto serial = ctx.serial;
      size_t at = 1;
      if (tokens.size() >= 3)
      {
        serial = tokens[1];
        at = 2;
      }
      auto req = std::make_shared<AssignInstantActions::Request>();
      req->manufacturer = ctx.mfg;
      req->serial_number = serial;
      req->instant_actions.header.version = "2.0.0";
      req->instant_actions.header.manufacturer = ctx.mfg;
      req->instant_actions.header.serial_number = serial;
      vda5050_interfaces::msg::Action a;
      a.action_type = tokens[at];
      a.action_id = fmt::format(
        "ia-{}", std::chrono::steady_clock::now().time_since_epoch().count());
      a.blocking_type = "NONE";
      req->instant_actions.actions.push_back(a);
      auto r = call_service<AssignInstantActions>(
        ctx.node, fmt::format("/{}/assign_instant_actions", ctx.master_ns), req,
        ctx.timeout);
      if (r) fmt::print("  decision={}\n", static_cast<int>(r->decision));
      continue;
    }
    if (cmd == "resume")
    {
      auto req = std::make_shared<ResumeQueue::Request>();
      req->manufacturer = ctx.mfg;
      req->serial_number = arg_serial(1);
      auto r = call_service<ResumeQueue>(
        ctx.node, fmt::format("/{}/resume_mode_cancelled_queue", ctx.master_ns),
        req, ctx.timeout);
      if (r)
      {
        fmt::print(
          "  status={} orders_resumed={} actions_resumed={}\n",
          static_cast<int>(r->status), r->orders_resumed, r->actions_resumed);
      }
      continue;
    }
    if (cmd == "discard")
    {
      auto req = std::make_shared<DiscardQueue::Request>();
      req->manufacturer = ctx.mfg;
      req->serial_number = arg_serial(1);
      auto r = call_service<DiscardQueue>(
        ctx.node,
        fmt::format("/{}/discard_mode_cancelled_queue", ctx.master_ns), req,
        ctx.timeout);
      if (r)
      {
        fmt::print(
          "  status={} orders_discarded={} actions_discarded={}\n",
          static_cast<int>(r->status), r->orders_discarded,
          r->actions_discarded);
      }
      continue;
    }
    if (cmd == "monitor")
    {
      auto saved_serial = ctx.serial;
      if (tokens.size() > 1) ctx.serial = tokens[1];
      run_monitor(ctx);
      ctx.serial = saved_serial;
      g_running = true;  // Ctrl-C inside monitor returns us to REPL
      continue;
    }
    fmt::print("  ?? unknown command (try `help`)\n");
  }
  return 0;
}

// ============================================================================
// CLI parsing
// ============================================================================

struct Args
{
  std::string mfg = kDefaultMfg;
  std::string serial = kDefaultSerial;
  std::string master_ns = kDefaultMaster;
  std::chrono::milliseconds timeout{kDefaultTimeoutMs};
  enum class Mode
  {
    INTERACTIVE,
    SCENARIO,
    MONITOR
  } mode = Mode::INTERACTIVE;
  std::string scenario;
};

Args parse_args(int argc, char** argv)
{
  Args a;
  for (int i = 1; i < argc; ++i)
  {
    std::string tok = argv[i];
    auto next = [&](const char* flag) -> std::string {
      if (i + 1 >= argc)
      {
        fmt::print(stderr, "{} requires an argument\n", flag);
        std::exit(2);
      }
      return argv[++i];
    };
    if (tok == "--mfg")
      a.mfg = next("--mfg");
    else if (tok == "--serial")
      a.serial = next("--serial");
    else if (tok == "--master-ns")
      a.master_ns = next("--master-ns");
    else if (tok == "--timeout-ms")
      a.timeout = std::chrono::milliseconds(std::stoi(next("--timeout-ms")));
    else if (tok == "--scenario")
    {
      a.mode = Args::Mode::SCENARIO;
      a.scenario = next("--scenario");
    }
    else if (tok == "--interactive")
      a.mode = Args::Mode::INTERACTIVE;
    else if (tok == "--monitor")
    {
      a.mode = Args::Mode::MONITOR;
      a.serial = next("--monitor");
    }
    else if (tok == "--help" || tok == "-h")
    {
      fmt::print(
        "Usage: mock_fms [options]\n"
        "  --mfg STR          AGV manufacturer (default {})\n"
        "  --serial STR       AGV serial (default {})\n"
        "  --master-ns STR    master namespace (default {})\n"
        "  --timeout-ms N     per-call timeout (default {})\n"
        "  --scenario S       happy-path|stitch|broker-bounce|"
        "onboard-flood\n"
        "  --interactive      REPL (default)\n"
        "  --monitor SERIAL   monitor one AGV's topics\n",
        kDefaultMfg, kDefaultSerial, kDefaultMaster, kDefaultTimeoutMs);
      std::exit(0);
    }
    else
    {
      fmt::print(stderr, "unknown flag '{}'; try --help\n", tok);
      std::exit(2);
    }
  }
  return a;
}

}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  Args a = parse_args(argc, argv);
  rclcpp::init(argc, argv);
  FmsContext ctx;
  ctx.node = rclcpp::Node::make_shared("mock_fms");
  ctx.mfg = a.mfg;
  ctx.serial = a.serial;
  ctx.master_ns = a.master_ns;
  ctx.timeout = a.timeout;

  int rc = 0;
  switch (a.mode)
  {
    case Args::Mode::SCENARIO:
      rc = run_scripted(ctx, a.scenario);
      break;
    case Args::Mode::MONITOR:
      rc = run_monitor(ctx);
      break;
    case Args::Mode::INTERACTIVE:
      rc = run_repl(ctx);
      break;
  }

  rclcpp::shutdown();
  return rc;
}
