# Master API Reference

Every command you can call on `VDA5050Master`, and every event it can call you
back on. For the guided walkthrough, start with [`master.md`](master.md).

All examples assume:

```cpp
#include "vda5050_core/master/master.hpp"

using vda5050_core::master::VDA5050Master;
namespace types = vda5050_core::types;

auto mqtt = vda5050_core::transport::create_default_client_shared(
  "tcp://localhost:1883", "my_master");
auto master = VDA5050Master::make(mqtt);
```

## Contents

- [Commands](#commands)
  - [Lifecycle](#lifecycle)
  - [Managing AGVs](#managing-agvs)
  - [Sending work](#sending-work)
  - [Recovering work](#recovering-work)
  - [Layout](#layout)
- [Types](#types)
- [Callbacks](#callbacks)
  - [Messages](#messages)
  - [Order progress](#order-progress)
  - [Availability](#availability)
  - [Faults and mode](#faults-and-mode)
  - [Broker](#broker)

## Commands

### Lifecycle

```cpp
// Create the master. The only constructor; shared ownership is required.
auto master = VDA5050Master::make(mqtt);

// Open the broker connection. Register callbacks before this.
master->connect();

// Is the broker connection up right now?
if (!master->is_connected()) { /* your code */ }

// Connection detail, including how many times it has reconnected.
auto status = master->get_broker_status();
// status.connected, status.last_disconnect_at, status.reconnect_count

// Close it. Queued work stops; the master can be reconnected.
master->disconnect();
```

### Managing AGVs

```cpp
// Start managing an AGV. Subscribes to its topics; until this, its
// messages are ignored. Default interface name is "uagv".
master->onboard_agv("Manufacturer", "S001");

// With a custom interface name, queue size, and full-queue policy.
master->onboard_agv("uagv", "Manufacturer", "S001", 10, true);

// A whole roster at once, under one lock.
std::vector<VDA5050Master::OnboardSpec> specs = {
  {"Manufacturer", "S001", 10, true},
  {"Manufacturer", "S002", 10, true},
};
auto result = master->onboard_agv_batch(specs);
// result.onboarded, result.skipped_already_onboarded, result.failed

// Stop managing one, or several.
master->offboard_agv("Manufacturer", "S001");
std::size_t removed = master->offboard_agv_batch(
  {{"Manufacturer", "S001"}, {"Manufacturer", "S002"}});

// Who is being managed?
if (master->is_agv_onboarded("Manufacturer", "S001")) { /* your code */ }

for (const auto& [manufacturer, serial] : master->get_onboarded_agvs())
{
  // your code: iterate the fleet
}

// Read-only view of one AGV's cached state.
auto agv = master->get_agv("Manufacturer", "S001");
if (agv)
{
  auto state = agv->get_last_state();          // std::optional<types::State>
  auto op_state = agv->get_operational_state(); // AVAILABLE, UNAVAILABLE, ...
  auto conn = agv->get_connection_status();     // ONLINE, OFFLINE, ...
}
```

### Sending work

```cpp
// Validate and queue an order. Returns once queued, not once published.
auto res = master->assign_order("Manufacturer", "S001", order);
if (res.decision != vda5050_core::master::OrderAssignmentDecision::ASSIGNED)
{
  for (const auto& e : res.errors)
  {
    // your code: report e.error_type, e.error_description
  }
}

// Validate and queue instant actions. Validated synchronously.
types::InstantActions ia;
ia.actions = {vda5050_core::master::ActionFactory::build_state_request(
  vda5050_core::master::ActionFactory::generate_action_id())};
auto ia_res = master->assign_instant_actions("Manufacturer", "S001", ia);
```

`publish_order` and `publish_instant_actions` take the same arguments but skip
validation and header filling, returning only `bool`. The caller owns
correctness — prefer the `assign_*` pair.

See [what the master validates](master.md#4-what-the-master-validates) for the
order of the checks and which rejections reach the caller.

### Recovering work

```cpp
// Drop an AGV's queued orders and instant actions. Does NOT tell the AGV to
// stop — send a cancelOrder instant action for that.
master->cancel_pending_orders("Manufacturer", "S001");

// When an AGV leaves master control, its un-sent work is drained to a buffer.
// Put it back at the front of the live queue:
auto [orders, actions] =
  master->resume_mode_cancelled_queue("Manufacturer", "S001");

// Or throw it away:
auto [dropped_orders, dropped_actions] =
  master->discard_mode_cancelled_queue("Manufacturer", "S001");
```

### Layout

```cpp
// Load a LIF topology. Enables node/edge existence, map id, and edge
// direction checks during order validation.
auto load = master->load_layout_from_config("layout.json");
if (!load)
{
  // your code: report load.errors
}

// Install an already-built graph instead.
master->set_graph(graph);

// The graph in use, or nullptr. Safe to hold across swaps.
auto graph = master->get_loaded_graph();

// Re-check one AGV's factsheet against the layout. Prefer reacting in
// on_factsheet over calling this directly.
master->refresh_alignment_for_agv("Manufacturer/S001", factsheet);

// Every AGV's alignment result, keyed by agv_id.
auto cache = master->get_alignment_cache_snapshot();
```


## Types

The vocabulary the commands above trade in.

### `OrderAssignmentDecision`

Returned by `assign_order` as `res.decision`. Branch on this, not on
`operator bool` — that is `ASSIGNED`-only, so the two non-failures below test
false while carrying no errors.

| Value | Means | Typical response |
| --- | --- | --- |
| `ASSIGNED` | queued for publish | carry on |
| `STITCH_QUEUED` | update held until the AGV confirms the previous one; publishes itself later | nothing — not a failure |
| `DUPLICATE_IGNORED` | same `order_update_id` already applied | nothing — not a failure |
| `AGV_NO_STATE_YET` | no State received from the AGV yet | retry later |
| `AGV_NOT_READY` | operational state is `ERROR` / `UNAVAILABLE` / `STATE_UNKNOWN` | retry later |
| `AGV_OFFLINE` | connection is not `ONLINE` | retry on reconnect |
| `AGV_QUEUE_FULL` | outbound queue is full; the connection is fine | retry when it drains |
| `AGV_MODE_NOT_AUTO` | AGV is not under master control | operator intervention |
| `AGV_POSITION_NOT_INITIALIZED` | AGV has not localized | send an `initPosition`, then retry |
| `AGV_NOT_ONBOARDED` | no AGV with that manufacturer/serial | fix the identity, or onboard it |
| `STITCH_REJECTED` | backward `order_update_id`, or the update does not stitch onto the active order | fix the order — retrying identically fails identically |

### `OrderAssignmentResult`

| Member | Holds |
| --- | --- |
| `decision` | one of the values above |
| `errors` | `std::vector<types::Error>` — empty for `ASSIGNED`, `STITCH_QUEUED`, `DUPLICATE_IGNORED` |
| `operator bool` | `true` only when `decision == ASSIGNED` |

Each `types::Error` carries `error_type`, `error_level`, an optional
`error_description`, and optional references to the offending field.

### `InstantActionDecision`

Returned by `assign_instant_actions`.

| Value | Means |
| --- | --- |
| `ASSIGNED` | queued for publish |
| `AGV_NOT_ONBOARDED` | no AGV with that manufacturer/serial |
| `AGV_OFFLINE` | connection is not `ONLINE` |
| `AGV_QUEUE_FULL` | outbound queue is full |
| `INVALID_CONTENT` | failed schema validation, or the message has no actions |
| `DUPLICATE_ACTION_ID` | an `action_id` collides with one in flight, in the active order, queued, or repeated in the message |
| `AGV_MODE_NOT_AUTO_FOR_ACTION` | AGV is not under master control and the action type is not exempt |
| `AGV_CANNOT_PERFORM_ACTION` | action type is not in the AGV's factsheet |
| `ACTION_BLOCKED_BY_DRIVING` | `SOFT` or `HARD` blocking action while the AGV is driving |
| `HARD_ACTION_BLOCKED` | `HARD` blocking action while another action is running |
| `EXCEEDS_PROTOCOL_LIMITS` | exceeds an array size the AGV declared in its factsheet |

`InstantActionAssignmentResult` has the same shape as
`OrderAssignmentResult` — `decision`, `errors`, and an `ASSIGNED`-only
`operator bool`.

### `AGVState`

From `agv->get_operational_state()`.

| Value | Means |
| --- | --- |
| `STATE_UNKNOWN` | no State yet, or the heartbeat timed out |
| `AVAILABLE` | State is arriving; the AGV is operational |
| `UNAVAILABLE` | the AGV reported unavailable, or the connection dropped |
| `ERROR` | reserved; not currently set |

### `BrokerStatusSnapshot`

From `get_broker_status()`.

| Member | Holds |
| --- | --- |
| `connected` | broker connection state |
| `last_disconnect_at` | `std::optional<system_clock::time_point>`; unset if never dropped |
| `reconnect_count` | `1` after the first connect, incremented on each reconnect |

### `OnboardSpec` and `BatchOnboardResult`

For `onboard_agv_batch`.

| `OnboardSpec` | Holds |
| --- | --- |
| `manufacturer`, `serial_number` | the AGV's identity |
| `max_queue_size` | outbound queue cap (default 10) |
| `drop_oldest` | on a full queue, drop the oldest rather than reject the new one |

`BatchOnboardResult` splits the input into `onboarded`,
`skipped_already_onboarded`, and `failed` (empty manufacturer or serial).

### `PoseView` and `PoseSource`

From `agv->get_pose_view()`, the freshest pose across State and Visualization.

| Member | Holds |
| --- | --- |
| `source` | `None`, `State`, or `Visualization` |
| `agv_position`, `velocity` | optional; empty when `source == None` |
| `driving` | whether the AGV reports itself moving |
| `data_age` | how long since the underlying message arrived |

`source == None` means no initialized position is cached — treat the pose as
unknown rather than stale.

## Callbacks

Register all of these **before** `connect()`. Each event has one slot —
registering twice replaces the first. They all run on the same inbound thread,
so keep them prompt.

### Messages

Every inbound message, before any interpretation.

```cpp
master->on_state(
  [&](const std::string& agv_id, const types::State& state) {
    // your code: telemetry, dashboards, anything driven by raw State
  });

master->on_connection(
  [&](const std::string& agv_id, const types::Connection& connection) {
    // your code: raw connection message
  });

master->on_factsheet(
  [&](const std::string& agv_id, const types::Factsheet& factsheet) {
    // your code: record capabilities, limits, speed envelope
  });

master->on_visualization(
  [&](const std::string& agv_id, const types::Visualization& visualization) {
    // your code: high-rate pose updates
  });
```

### Order progress

```cpp
master->on_node_reached(
  [&](const std::string& agv_id, const std::string& node_id) {
    // your code: update where you think the robot is
  });

master->on_order_complete(
  [&](const std::string& agv_id, const std::string& order_id) {
    // your code: assign the next task for this AGV
  });

master->on_new_base_requested([&](const std::string& agv_id) {
  // your code: the AGV wants more base nodes; send an order update
});
```

### Availability

```cpp
master->on_connect([&](const std::string& agv_id) {
  // your code: the AGV came online; request a factsheet, localize it
});

master->on_offline([&](const std::string& agv_id) {
  // your code: it said goodbye cleanly; mark it out of service
});

master->on_connection_broken([&](const std::string& agv_id) {
  // your code: it dropped without warning; requeue its work
});

master->on_state_timeout([&](const std::string& agv_id) {
  // your code: no State for 30s; orders will be rejected until it resumes
});

master->on_state_resumed([&](const std::string& agv_id) {
  // your code: State is flowing again, or arrived for the first time
});
```

### Faults and mode

```cpp
master->on_errors_appeared(
  [&](const std::string& agv_id, const std::vector<types::Error>& new_errors) {
    // your code: raise operator alerts for the newly-appeared errors only
  });

master->on_errors_resolved(
  [&](const std::string& agv_id, const std::vector<types::Error>& resolved) {
    // your code: clear the alerts that no longer apply
  });

master->on_mode_changed(
  [&](const std::string& agv_id, types::OperatingMode now,
      types::OperatingMode before) {
    // your code: leaving AUTOMATIC drains un-sent work to a buffer;
    // resume_mode_cancelled_queue puts it back
  });

master->on_paused([&](const std::string& agv_id, bool paused) {
  // your code: the AGV's paused flag flipped
});

master->on_driving([&](const std::string& agv_id, bool driving) {
  // your code: the AGV started or stopped moving
});

master->on_loads_changed(
  [&](const std::string& agv_id, const std::vector<types::Load>& loads) {
    // your code: the full current load list, empty if carrying nothing
  });
```

### Broker

The master's own connection, not any AGV's. No `agv_id` argument.

```cpp
master->on_broker_disconnected([&]() {
  // your code: fleet-wide degradation; queued orders wait for reconnect
});

master->on_broker_reconnected([&]() {
  // your code: fires on the first connect too, not only on reconnects
});
```
