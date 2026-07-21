# Validation

This document describes the validators in `vda5050_core::validation` and how
the master composes them when publishing orders and instant actions.

For building a master, start with [`master.md`](master.md);
[`master-api.md`](master-api.md) is the command and callback reference.

## 1. Overview

Every validator is a free function in `vda5050_core::validation` returning
`vda5050_core::errors::ValidationResult`. They are stateless and safe to call
concurrently, and each one is usable on its own.

Findings come at two levels:

- **Fatal** — the caller treats the message as rejected.
- **Warning** — advisory. The message is still published.

```cpp
#include "vda5050_core/validation/content_validator.hpp"

auto result = vda5050_core::validation::validate_order_content(order);
if (!result)
{
  // fatal errors only; warnings do not set this
}
```

## 2. ValidationResult

| Member | Returns |
| --- | --- |
| `add_error(error)` | files an entry by level |
| `fatal_errors()` | entries that block publishing |
| `warnings()` | advisory entries |
| `has_fatal()` | `true` if any fatal entry |
| `has_warnings()` | `true` if any warning entry |
| `operator bool` | `true` when there are **no fatal** entries |

Each entry is a `types::Error` carrying an error type and level; the description
and field references are optional.

> **`operator bool` ignores warnings.** `if (result)` is `true` for a result
> holding warnings and no fatal errors. Test `has_warnings()` when you need to
> know whether a check actually ran.

## 3. The validators

One header per concern, in `include/vda5050_core/validation/`.

### `content_validator.hpp`

Required-content checks per message type: header version, manufacturer and
serial non-empty, ids non-empty, and every action carrying an `action_id` and
`action_type`.

| Function | Validates |
| --- | --- |
| `validate_order_content` | Order |
| `validate_instant_actions_content` | InstantActions |
| `validate_state_content` | State |
| `validate_connection_content` | Connection |
| `validate_factsheet_content` | Factsheet |
| `validate_visualization_content` | Visualization |

All findings are fatal. These are the only checks that need nothing but the
message itself.

### `pre_send_validator.hpp`

`validate_pre_send` is the AGV-readiness gate. It takes a `PreSendContext`
snapshot (connection status, last State, last Factsheet, operational state,
loaded graph) and fails, fatally, when the AGV:

- is not `ONLINE`
- is not in operational state `AVAILABLE`
- has not yet reported any State
- reports a mode other than `AUTOMATIC` or `SEMIAUTOMATIC`
- is paused
- has its e-stop engaged
- reports no position, or `positionInitialized: false`

### `order_graph_validator.hpp`

`is_valid_graph` checks that an order's nodes and edges form a valid graph:
sequence numbering, node/edge alternation, released-versus-horizon consistency.

`is_valid_update(base_order, next_order)` checks a sparse update against the
order it extends: same `orderId`, a newer `orderUpdateId`, and a first node
that matches the base's decision point.

Both are fatal throughout, so `operator bool` is enough to gate on.

> The `\note` on `is_valid_graph` in the header still describes these findings
> as advisory `WARNING` level and tells you to gate on
> `has_fatal() || has_warnings()`. That is out of date — every finding from
> both functions is `FATAL`.

The two functions suit different callers. `is_valid_update` needs both orders
in full, which only a client holds: the master merges the update and runs
`is_valid_graph` on the merged order instead, validating what it actually
sends.

### `traversability_validator.hpp`

`validate_traversability` checks that this AGV can actually drive the route:
first-node reachability, and — when a layout is loaded — node and edge
existence, map id, and edge direction.

Fatal. With no State reported it fails hard rather than skipping. With no layout
loaded it runs reachability only and warns that the layout checks were skipped.

### `capability_validator.hpp`

`validate_capability` checks the message's actions against the AGV factsheet:
action type supported, required scope, blocking type, declared and required
parameters. Overloads exist for Order and InstantActions; only the
InstantActions overload exempts the predefined action types.

Fatal, but it needs a factsheet — see *Checks that may not run*.

### `protocol_limits_validator.hpp`

`validate_protocol_limits` checks array sizes against the factsheet's declared
limits: `order_nodes` and `order_edges` for the route, `node_actions` and
`edge_actions` per element, `actions_actions_parameters` per action, and
`instant_actions` per message. Overloads exist for Order and InstantActions.

Fatal, but it needs a factsheet — see *Checks that may not run*. The array
limits are per-order, so on a stitch the subject is the merged order, not the
update fragment.

### `instant_action_mode_validator.hpp`

`validate_instant_action_mode` gates instant actions on the operating mode.
While the master is in control — `AUTOMATIC` or `SEMIAUTOMATIC` — everything
passes. Otherwise, and when no State has been received, only exempt action types
get through. `is_mode_exempt_action_type` reports which.

### `action_conflict_validator.hpp`

`validate_action_conflict` checks an instant action against the AGV's running
actions and driving state:

| Blocking type | Rejected when |
| --- | --- |
| `NONE` | — |
| `SOFT` | driving |
| `HARD` | driving, or any action is active |

Two exceptions: `initPosition` is rejected while driving whatever its blocking
type, and `cancelOrder` / `startPause` / `stopPause` bypass the check entirely.
With no cached State the whole check passes.

### `factsheet_alignment.hpp`

`check_factsheet_alignment` compares the loaded layout's edge speeds against the
AGV factsheet's speed envelope. **Warning** level throughout — it is a
diagnostic, not a gate. The master runs it when a factsheet arrives.

### `operating_mode_control.hpp` and `predefined_action_types.hpp`

Helpers that answer a question rather than validate:

| Helper | Returns |
| --- | --- |
| `is_master_in_control(mode)` | mode is `AUTOMATIC` or `SEMIAUTOMATIC` |
| `is_mode_exempt_action_type(type)` | action passes the operating-mode gate |
| `is_capability_exempt_action_type(type)` | action needs no factsheet entry (`charge` is not exempt) |
| `is_motion_exempt_action_type(type)` | action may be issued mid-motion |
| `is_position_init_action_type(type)` | action rewrites the pose |

### Checks that may not run

Three checks need data that may not have arrived:

| Check | Needs | Warns with |
| --- | --- | --- |
| `validate_capability` | the AGV factsheet | `CapabilityCheckSkipped` |
| `validate_protocol_limits` | the AGV factsheet | `ProtocolLimitCheckSkipped` |
| `validate_traversability` (layout part) | a loaded layout | `GraphIntegrityCheckSkipped` |

When the data is missing the check does not run and a warning records that.
Since warnings do not set `operator bool`, a result with no fatal errors is not
proof the check ran. Request a factsheet on connect if you rely on either
factsheet-gated check.

## 4. Who uses what

Every validator is usable on its own, but the two sides of the protocol reach
for different ones. The master validates what it is about to **send**, so it
uses the whole set. A client validates what it has **received**, which only
needs the graph checks.

| Validator | Master | Client |
| --- | --- | --- |
| `is_valid_graph` | yes | yes |
| `is_valid_update` | no | yes |
| `validate_*_content` | yes | no |
| `validate_pre_send` | yes | no |
| `validate_traversability` | yes | no |
| `validate_capability` | yes | no |
| `validate_protocol_limits` | yes | no |
| `validate_instant_action_mode` | yes | no |
| `validate_action_conflict` | yes | no |
| `check_factsheet_alignment` | yes | no |

The client-side checks the shared validators do **not** cover — is the vehicle
busy, is this update a duplicate, does the stitch node match the decision
point — are implemented by each client in its own acceptance logic rather than
in `vda5050_core::validation`.

## 5. Error types

Each entry in a `ValidationResult` carries an error type naming the check that
produced it.

| Error type | Raised by |
| --- | --- |
| `ContentValidationError` | the content validators |
| `PreSendValidationError` | `validate_pre_send` |
| `GraphValidationError` | `is_valid_graph` |
| `OrderUpdateError` | `is_valid_update`, order stitching |
| `TraversabilityValidationError` | `validate_traversability` |
| `CapabilityValidationError` | `validate_capability` |
| `ProtocolLimitError` | `validate_protocol_limits` |
| `ModeValidationError` | `validate_instant_action_mode` |
| `ActionBlockedByDrivingError`, `HardActionBlockedError` | `validate_action_conflict` |
| `ValidationError` | callers, for conditions outside any single validator |

Most entries also carry field references identifying what failed, such as
`RefOrderId`, `RefNodeId`, `RefSequenceId`, or `RefActionId`.

For how the master chains these when publishing, and which rejections reach the
caller, see [`master.md`](master.md).
