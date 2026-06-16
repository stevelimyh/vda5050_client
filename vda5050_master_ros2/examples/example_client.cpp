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

// example_client — VDA5050 AGV-side reference exec for interop testing.
//
// Structure:
//   SECTION 1  Verbatim port of Saurabh's example_client (throwaway/test-client
//              @ b93a4ae), mechanical changes only: includes / namespaces /
//              API calls updated for the current single-package layout.
//              Preserves his Handler/Strategy framework code (NavigationStrategy,
//              SimpleContext, OrderUpdate / NodeDispatchEvent / NodeAckUpdate).
//   SECTION 2  TEST-ONLY EXTENSION: MinimalStatePublisher. Not part of
//              Saurabh's upstream. Bridges Saurabh's Strategy framework to
//              the master's pre_send_validator by (a) publishing Connection +
//              State on the wire format the master subscribes to, and (b)
//              acknowledging NodeDispatchEvent so the framework progresses
//              through an order. A real AGV would have this driven by real
//              hardware (encoders / SLAM / motion controller).
//   SECTION 3  main(): wires SECTION 1 (Saurabh's framework) + SECTION 2
//              (our extension) together, drives the executor.
//
// Run:
//   ros2 run vda5050_master_ros2 example_client

#include <fmt/core.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

// ============================================================================
// SECTION 1 — Verbatim port of Saurabh's example_client.cpp
// ============================================================================
// Mechanical port mapping vs throwaway/test-client @ b93a4ae:
//   <vda5050_core/mqtt_client/...>     → <vda5050_core/transport/...>
//   <vda5050_types/...>                → <vda5050_core/types/...>
//   "vda5050_execution/..."            → "vda5050_core/execution/..."
//   vda5050_execution::*               → vda5050_core::execution::*
//   vda5050_core::mqtt_client::*       → vda5050_core::transport::*
//   vda5050_types::*                   → vda5050_core::types::*
//   create_default_client_unique()     → create_default_client_shared() (shared_ptr)
//   ProtocolAdapter::connect()         → MqttClientInterface::connect() (raw)
//   ProtocolAdapter::set_will<T>()     → MqttClientInterface::set_will(topic,
//                                          json, qos) (raw, before connect)
// Constants "uagv"/"v2" match the master's topic convention
// (vda5050_core/master/standard_names.hpp). Header.version for outbound
// messages is handled by SECTION 2 to satisfy schema validator.

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/handler.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/execution/strategy_interface.hpp"
#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/e_stop.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/state.hpp"

using vda5050_core::execution::ContextInterface;
using vda5050_core::execution::EventBase;
using vda5050_core::execution::Handler;
using vda5050_core::execution::Initialize;
using vda5050_core::execution::Priority;
using vda5050_core::execution::ProtocolAdapter;
using vda5050_core::execution::ResourceBase;
using vda5050_core::execution::StrategyInterface;
using vda5050_core::execution::UpdateBase;
using vda5050_core::transport::MqttClientInterface;

std::atomic_bool g_running{true};

void signal_handler(int signal)
{
  VDA5050_INFO_STREAM(
    "System Signal [" << signal << "] received. Shutting down ...");
  g_running = false;
}

struct OrderUpdate : public Initialize<OrderUpdate, UpdateBase>
{
  vda5050_core::types::Order order;

  explicit OrderUpdate(const vda5050_core::types::Order& order) : order(order)
  {
    // Nothing to do here ...
  }
};

struct NodeDispatchEvent : public Initialize<NodeDispatchEvent, EventBase>
{
  uint32_t sequence_id;
  std::string node_id;
  double x, y;
  std::optional<double> theta;

  NodeDispatchEvent(
    uint32_t sequence_id, const std::string& node_id, double x, double y,
    std::optional<double> theta = std::nullopt)
  : sequence_id(sequence_id), node_id(node_id), x(x), y(y), theta(theta)
  {
    // Nothing to do here ...
  }
};

struct NodeAckUpdate : public Initialize<NodeAckUpdate, UpdateBase>
{
  uint32_t sequence_id;

  explicit NodeAckUpdate(uint32_t sequence_id) : sequence_id(sequence_id)
  {
    // Nothing to do here ...
  }
};

class NavigationStrategy : public StrategyInterface
{
public:
  void init(std::shared_ptr<ContextInterface> context) override
  {
    VDA5050_INFO("Initializing NavigationStrategy ...");

    engine()->on<NodeDispatchEvent>([](auto event) {
      VDA5050_INFO_STREAM(
        "Navigating to [" << event->x << ", " << event->y << "] "
                          << "with sequence " << event->sequence_id);
    });

    context->provider()->on<NodeAckUpdate>(
      [w = std::weak_ptr(engine())](auto update) {
        if (auto m = w.lock()) m->notify(update);
      });
  }

  void step(std::shared_ptr<ContextInterface> context) override
  {
    if (engine()->waiting()) return;

    if (nodes_.empty())
    {
      auto order_update = context->get_update<OrderUpdate>();
      if (order_update)
      {
        // Saurabh's original step() would re-fetch the same cached
        // OrderUpdate every tick after finishing, looping "Adding nodes"
        // forever. Guard by (order_id, order_update_id) so each order
        // is walked once. An order *update* (same id, higher update_id)
        // re-enters this branch — current_idx_ resets to 0 but the
        // shared-state MinimalStatePublisher::advance_idx_ keeps prior
        // progress, so already-walked nodes don't re-fire ack events.
        const auto& o = order_update->order;
        if (
          o.order_id == last_processed_order_id_ &&
          o.order_update_id == last_processed_update_id_)
        {
          return;
        }
        VDA5050_INFO("Adding nodes");
        nodes_ = o.nodes;
        current_idx_ = 0;
        last_processed_order_id_ = o.order_id;
        last_processed_update_id_ = o.order_update_id;
      }
      else
      {
        return;
      }
    }

    if (current_idx_ < nodes_.size())
    {
      auto target = nodes_[current_idx_++];
      // Skip horizon nodes — they are preview-only per §6.6.4. Saurabh's
      // original walked every node; this guard keeps the AGV parked at
      // the last released base node and only advances when an order
      // update releases the next segment.
      if (!target.released)
      {
        nodes_.clear();
        current_idx_ = 0;
        return;
      }
      const double x = target.node_position.value().x;
      const double y = target.node_position.value().y;
      const double theta = target.node_position.value().theta.value_or(0.0);
      engine()->emit<NodeDispatchEvent>(
        Priority::NORMAL, target.sequence_id, target.node_id, x, y, theta);

      VDA5050_INFO(
        "Pushing node with sequence_id [{}] to event queue",
        target.sequence_id);

      engine()->step();

      // Current engine API: `suspend_for` requires a timeout. 60 s is
      // generous; the MinimalStatePublisher (SECTION 2) acks immediately
      // on NodeDispatchEvent, so the wait is effectively instantaneous.
      engine()->suspend_for<NodeAckUpdate>(
        std::chrono::seconds(60),
        [seq = target.sequence_id](auto update) -> bool {
          return update->sequence_id == seq;
        });
    }
    else
    {
      nodes_.clear();
    }
  }

private:
  std::vector<vda5050_core::types::Node> nodes_;
  size_t current_idx_ = 0;
  std::string last_processed_order_id_;
  uint32_t last_processed_update_id_ = 0;
};

// Saurabh's original StateStrategy was a stub that did nothing — see the
// `class StateStrategy` in throwaway/test-client. We replace it with
// MinimalStatePublisher in SECTION 2, which actually publishes State on
// the wire. Saurabh's stub is intentionally NOT carried over.

class SimpleContext : public ContextInterface,
                      public std::enable_shared_from_this<SimpleContext>
{
public:
  void init() override
  {
    provider()->on<OrderUpdate>([w = weak_from_this()](auto update) {
      if (auto m = w.lock())
      {
        std::lock_guard<std::mutex> lock(m->mutex_);
        m->updates_[update->get_type()] = update;
      }
    });
  }

protected:
  std::shared_ptr<UpdateBase> get_update_raw(
    std::type_index type) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return (updates_.count(type)) ? updates_.at(type) : nullptr;
  }

  std::shared_ptr<ResourceBase> get_resource_raw(
    std::type_index /*type*/) const override
  {
    return nullptr;
  }

private:
  std::unordered_map<std::type_index, std::shared_ptr<UpdateBase>> updates_;
  mutable std::mutex mutex_;
};

// ============================================================================
// SECTION 2 — TEST-ONLY EXTENSION: MinimalStatePublisher
// ============================================================================
//
// THIS SECTION IS NOT PART OF SAURABH'S UPSTREAM example_client.
//
// Why it exists:
//   - The master's pre_send_validator (vda5050_core::master::pre_send_*)
//     rejects every AssignOrder with AGV_NO_STATE_YET until it sees a State
//     message from the AGV. Saurabh's StateStrategy was stubbed (no actual
//     publish) so the master never moves past pre-flight.
//   - The master also requires Connection ONLINE before accepting orders;
//     Saurabh's main() publishes one but via ProtocolAdapter, which fills
//     the header with version="v2" (the topic-path version constant). The
//     master's schema validator requires header.version="2.0.0". Same
//     workaround we already use in mock_client: bypass the typed publish
//     and emit raw JSON with the correct header.version.
//   - The Strategy framework's NavigationStrategy `suspend`s on
//     NodeAckUpdate, expecting an external motion controller to push the
//     ack. Without one, the AGV gets stuck on the first node. This class
//     immediately-acks NodeDispatchEvent so the order progresses (which
//     is enough for protocol-level testing; a real AGV would do real
//     motion).
//
// What a real AGV would do differently:
//   - Drive State publishing from real telemetry (odometry, SLAM, battery).
//   - Push NodeAckUpdate when actual motion completes, not synchronously.
//   - Set its own header.version per the spec version it implements.
//   - Run a proper safety supervisor for e_stop / field_violation.
//
// Threading:
//   - Constructor: registers as a Strategy. Each StrategyInterface has its
//     own Engine, so subscribing to NodeDispatchEvent via engine()->on()
//     would only see events emitted by *our* engine — not NavigationStrategy's.
//     Instead we drive advancement via the shared Provider channel: subscribe
//     to OrderUpdate (Provider broadcasts to all Strategies), then on each
//     tick advance one released node and push NodeAckUpdate (which the
//     Provider then routes to NavigationStrategy's suspend_for predicate).
//   - tick_loop thread (1 Hz): advance one released node + push NodeAckUpdate,
//     publish State + Connection heartbeat.
//   - state_mtx_ protects pose / order_id / last_node_* / active_order_ /
//     advance_idx_. Never held while calling mqtt_->publish or
//     provider->push (lock-order discipline mirrors mock_client).

constexpr const char* kInterface = "uagv";
constexpr const char* kTopicVersion = "v2";
constexpr const char* kProtocolVersion = "2.0.0";  // schema-validator-compat

class MinimalStatePublisher : public StrategyInterface
{
public:
  MinimalStatePublisher(
    std::shared_ptr<MqttClientInterface> mqtt, const std::string& mfg,
    const std::string& serial, int tick_ms = 1000)
  : mqtt_(mqtt),
    mfg_(mfg),
    serial_(serial),
    tick_ms_(tick_ms),
    topic_state_(
      fmt::format("{}/{}/{}/{}/state", kInterface, kTopicVersion, mfg, serial)),
    topic_conn_(fmt::format(
      "{}/{}/{}/{}/connection", kInterface, kTopicVersion, mfg, serial))
  {
    // Defaults: AGV at origin, AUTOMATIC, position initialised. The
    // master's pre_send checks require position_initialized + AUTOMATIC.
    pose_.position_initialized = true;
    pose_.x = 0.0;
    pose_.y = 0.0;
    pose_.theta = 0.0;
    pose_.map_id = "warehouse_floor1";
  }

  ~MinimalStatePublisher() override
  {
    // Defensive: if the operator forgot to call shutdown(), do the
    // minimum cleanup here. We MUST NOT call mqtt_->publish from a
    // destructor — at static / shared_ptr teardown the broker socket
    // may already be closed and Paho can throw, which would abort
    // the program. shutdown() is the supported path; this branch is
    // a safety net that only joins the thread.
    alive_ = false;
    if (tick_.joinable())
    {
      tick_.join();
    }
  }

  // Explicit cleanup: stop the tick thread, then publish graceful
  // OFFLINE while the MQTT client is still alive and connected.
  // Idempotent. Call from main() BEFORE the handler / mqtt teardown
  // (i.e. while example_client's main() still owns the shared_ptrs).
  void shutdown()
  {
    bool first =
      shutdown_called_.exchange(true, std::memory_order_acq_rel) == false;
    if (!first) return;
    alive_ = false;
    if (tick_.joinable())
    {
      tick_.join();
    }
    // Counterpart to the last-will: graceful OFFLINE (retained) while
    // the MQTT socket is still up.
    publish_connection(vda5050_core::types::ConnectionState::OFFLINE);
  }

  // Build the last-will + connect MQTT. Must be called BEFORE
  // mqtt_->connect() so the broker registers the will.
  void prepare_will_and_connect()
  {
    using vda5050_core::types::Connection;
    using vda5050_core::types::ConnectionState;
    Connection will;
    will.header = make_header(0);
    will.connection_state = ConnectionState::CONNECTIONBROKEN;
    nlohmann::json j = will;
    mqtt_->set_will(topic_conn_, j.dump(), /*qos=*/1);
    mqtt_->connect();
    publish_connection(ConnectionState::ONLINE);
  }

  // StrategyInterface — subscribe to OrderUpdate via the shared Provider
  // channel (engine() is per-Strategy, so we can't see NavigationStrategy's
  // NodeDispatchEvent emits from here). Tick thread drives advancement.
  void init(std::shared_ptr<ContextInterface> context) override
  {
    VDA5050_INFO("Initializing MinimalStatePublisher (test extension) ...");

    context->provider()->on<OrderUpdate>([this](auto update) {
      std::lock_guard<std::mutex> lock(state_mtx_);
      // Reset advance_idx_ on EVERY OrderUpdate. NavigationStrategy
      // re-walks every node from index 0 on each update (its
      // current_idx_=0 reset), so we must re-push NodeAckUpdate for
      // every sequence_id including ones we already walked — otherwise
      // NavigationStrategy stays suspended waiting for an ack of a
      // sequence_id we skipped past. State regression is guarded
      // separately in advance_one_node_locked (last_node_* / pose
      // only updates when the new sequence_id is greater than the
      // highest reported so far).
      active_order_ = update->order;
      active_order_id_ = update->order.order_id;
      active_order_update_id_ = update->order.order_update_id;
      advance_idx_ = 0;
    });

    if (!tick_.joinable())
    {
      tick_ =
        std::thread([this, ctx = std::weak_ptr<ContextInterface>(context)] {
          tick_loop(ctx);
        });
    }
  }

  void step(std::shared_ptr<ContextInterface> /*context*/) override
  {
    // No-op; state publishing + node advancement happens on tick_ thread.
  }

private:
  // Advance one node on this tick. Returns sequence_id to ack, or -1
  // if nothing to advance (no order, or parked at unreleased horizon).
  // Always pushes an ack for the next node in order (so NavigationStrategy
  // can unblock its suspend_for), but only mutates last_node_* / pose
  // forward — never regresses on already-walked sequence_ids. Reported
  // state therefore tracks the *highest* sequence reached, not the
  // most-recently-acked one.
  int64_t advance_one_node_locked()
  {
    if (!active_order_.has_value()) return -1;
    if (advance_idx_ >= active_order_->nodes.size()) return -1;
    const auto& node = active_order_->nodes[advance_idx_];
    if (!node.released)
    {
      return -1;  // park at last released node
    }
    // Only advance reported state when this sequence is forward progress.
    // Re-acks of already-walked nodes (e.g. NavigationStrategy re-walking
    // from index 0 after an order update) push the ack without regressing.
    if (last_node_id_.empty() || node.sequence_id > last_node_sequence_id_)
    {
      last_node_id_ = node.node_id;
      last_node_sequence_id_ = node.sequence_id;
      if (node.node_position.has_value())
      {
        pose_.x = node.node_position->x;
        pose_.y = node.node_position->y;
        pose_.theta = node.node_position->theta.value_or(0.0);
        if (!node.node_position->map_id.empty())
        {
          pose_.map_id = node.node_position->map_id;
        }
      }
    }
    advance_idx_++;
    return static_cast<int64_t>(node.sequence_id);
  }

  vda5050_core::types::Header make_header(uint32_t id)
  {
    vda5050_core::types::Header h;
    h.header_id = id;
    h.timestamp = std::chrono::system_clock::now();
    h.version = kProtocolVersion;  // "2.0.0" — schema-validator compat
    h.manufacturer = mfg_;
    h.serial_number = serial_;
    return h;
  }

  void publish_connection(vda5050_core::types::ConnectionState s)
  {
    vda5050_core::types::Connection c;
    c.header = make_header(conn_header_id_++);
    c.connection_state = s;
    nlohmann::json j = c;
    if (mqtt_)
      mqtt_->publish(topic_conn_, j.dump(), /*qos=*/1, /*retain=*/true);
  }

  void publish_state_snapshot()
  {
    // Build the entire snapshot — including header.timestamp — INSIDE
    // the lock so the (order_id, order_update_id, last_node_*, pose,
    // timestamp) tuple is atomic. Without this, a fresh OrderUpdate
    // arriving on the Provider thread between snapshotting node fields
    // and stamping the header would yield a State that claims
    // "at <stale node> as of <fresh time>" — the master's pre_send
    // validator uses both fields to decide stitch eligibility.
    vda5050_core::types::State snapshot;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      snapshot.order_id = active_order_id_;
      snapshot.order_update_id = active_order_update_id_;
      snapshot.last_node_id = last_node_id_;
      snapshot.last_node_sequence_id = last_node_sequence_id_;
      snapshot.agv_position = pose_;
      snapshot.header = make_header(state_header_id_++);
    }
    snapshot.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
    snapshot.driving = false;
    snapshot.paused = false;
    snapshot.battery_state.battery_charge = 95.0;
    snapshot.safety_state.e_stop = vda5050_core::types::EStop::NONE;
    snapshot.safety_state.field_violation = false;
    nlohmann::json j = snapshot;
    if (mqtt_) mqtt_->publish(topic_state_, j.dump(), /*qos=*/0);
  }

  void tick_loop(std::weak_ptr<ContextInterface> ctx)
  {
    while (alive_ && g_running)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(tick_ms_));
      if (!alive_ || !g_running) break;

      // Advance one released node per tick. Push NodeAckUpdate to the
      // shared Provider so NavigationStrategy's suspend_for predicate
      // (in its own engine) wakes and emits the next NodeDispatchEvent.
      int64_t ack_seq = -1;
      {
        std::lock_guard<std::mutex> lock(state_mtx_);
        ack_seq = advance_one_node_locked();
      }
      if (ack_seq >= 0)
      {
        if (auto c = ctx.lock())
        {
          VDA5050_INFO(
            "[MinimalStatePublisher] arrived at node seq={}; "
            "pushing NodeAckUpdate",
            ack_seq);
          c->provider()->push<NodeAckUpdate>(static_cast<uint32_t>(ack_seq));
        }
      }

      // Heartbeat Connection ONLINE every tick (§6.14 spec; also covers
      // the retained-message race we documented in mock_client).
      publish_connection(vda5050_core::types::ConnectionState::ONLINE);
      publish_state_snapshot();
    }
  }

  std::shared_ptr<MqttClientInterface> mqtt_;
  const std::string mfg_, serial_;
  const int tick_ms_;
  const std::string topic_state_;
  const std::string topic_conn_;

  std::atomic<uint32_t> conn_header_id_{0};
  std::atomic<uint32_t> state_header_id_{0};

  std::mutex state_mtx_;
  vda5050_core::types::AGVPosition pose_;
  std::string last_node_id_;
  uint32_t last_node_sequence_id_ = 0;
  std::string active_order_id_;
  uint32_t active_order_update_id_ = 0;
  std::optional<vda5050_core::types::Order> active_order_;
  size_t advance_idx_ = 0;

  std::thread tick_;
  std::atomic_bool alive_{true};
  std::atomic_bool shutdown_called_{false};
};

// ============================================================================
// SECTION 3 — main(): wire Saurabh's framework + MinimalStatePublisher
// ============================================================================

int main()
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  const std::string mfg = "Manufacturer";
  const std::string serial = "S001";

  // SECTION 1 plumbing — adapted: shared_ptr factory + uagv/v2 constants.
  // Client id must be unique per process: a hardcoded id causes Paho
  // MQTT to evict the prior connection when a fresh process reconnects,
  // producing a disconnect ping-pong that breaks back-to-back scenario
  // runs. Encode {serial, pid} so two example_clients with the same
  // serial (e.g. two scenarios run in sequence) never collide.
  auto mqtt_client = vda5050_core::transport::create_default_client_shared(
    "tcp://localhost:1883",
    fmt::format("vda5050_client-{}-{}", serial, getpid()));

  // SECTION 2 — Will + Connection ONLINE before the rest of the wiring,
  // so the master sees the AGV come online as soon as we connect.
  auto state_publisher =
    std::make_shared<MinimalStatePublisher>(mqtt_client, mfg, serial);
  state_publisher->prepare_will_and_connect();

  // Inbound order subscription via ProtocolAdapter (topic version "v2"
  // matches the master's outbound topic). The adapter only deserialises
  // incoming JSON; we don't care that it would also stamp header.version
  // with "v2" because we never publish through it.
  auto protocol_adapter =
    ProtocolAdapter::make(mqtt_client, kInterface, kTopicVersion, mfg, serial);

  auto context = std::make_shared<SimpleContext>();
  auto navigation_strategy = std::make_shared<NavigationStrategy>();
  auto handler = Handler::make(context, {navigation_strategy, state_publisher});

  protocol_adapter->subscribe<vda5050_core::types::Order>(
    [w = std::weak_ptr<ContextInterface>(context)](auto order, auto error) {
      if (error.has_value()) return;
      if (auto m = w.lock())
      {
        VDA5050_INFO("Received order with order_id: {}", order.order_id);
        m->provider()->push<OrderUpdate>(order);
      }
    },
    0);

  VDA5050_INFO(
    "[example_client] up: mfg={} serial={} interface={}/{} (header.version={})",
    mfg, serial, kInterface, kTopicVersion, kProtocolVersion);

  auto spin_thread = std::thread([&] { handler->spin(); });

  // Block until SIGINT/SIGTERM.
  while (g_running)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Ordered teardown:
  //   1. shutdown() stops the tick thread and publishes graceful OFFLINE
  //      while mqtt_ is still alive and connected. NEVER do this from
  //      the destructor — shared_ptr destruction order at exit is
  //      non-deterministic and the broker socket may already be closed.
  //   2. Join handler's spin thread (no new strategy work).
  //   3. shared_ptr release order then tears down handler / context /
  //      protocol_adapter / mqtt_client cleanly.
  state_publisher->shutdown();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  return 0;
}
