# vda5050_master_ros2

ROS 2 wrapper for the VDA5050 fleet master. Implements the FMS-side of
the VDA5050 v2.0.0 protocol: dispatches orders, validates them, tracks
per-AGV state, and exposes both an MQTT surface (to AGVs) and a ROS 2
surface (to FMS clients).

## What's in the package

| Component | Purpose |
|---|---|
| `VDA5050MasterROS2` library | Wraps `vda5050_core::master::VDA5050Master` + 12 ROS 2 services + 5 per-AGV topics + 4 global topics |
| `example_master` | Reference master executable — loads `sample_map.json`, advertises all endpoints |
| `mock_client` | Standalone single-AGV pure-MQTT stub (no rclcpp) for fault-injection testing |
| `example_client` | Ported VDA5050 client framework + `MinimalStatePublisher` for interop testing |
| `mock_fms` | FMS-side driver: scripted end-to-end scenarios with pass/fail exit codes |

## Endpoints

Default `<ns>` = `vda5050_master`; override via the `VDA5050MasterROS2`
ctor's `topic_namespace` argument for multi-master deployments.

### Per-AGV topics (6)

Created on first `Connection ONLINE` from the AGV. Continuous publish
streams — async observers subscribe without polling.

| Topic | Type | Purpose |
|---|---|---|
| `/<ns>/<mfg>/<serial>/state` | `vda5050_interfaces/State` | Raw VDA5050 State at AGV's publish rate |
| `/<ns>/<mfg>/<serial>/connection` | `vda5050_interfaces/Connection` | Connection state edges: `ONLINE` / `OFFLINE` / `CONNECTION_BROKEN` |
| `/<ns>/<mfg>/<serial>/factsheet` | `vda5050_interfaces/Factsheet` | AGV capability declaration (typeSpec, physicalParameters, etc.) |
| `/<ns>/<mfg>/<serial>/device_status` | `vda5050_master_ros2/DeviceStatus` | Combined snapshot — single-subscription convenience for consumers that want one topic instead of three |
| `/<ns>/<mfg>/<serial>/order_status` | `vda5050_master_ros2/OrderStatus` | Master's lifecycle view: phase + last_node + base/horizon counts + action_states + errors |
| `/<ns>/<mfg>/<serial>/pose_view` | `vda5050_master_ros2/PoseView` | Fused pose at a fixed rate (default 1 Hz, `VDA5050_POSE_VIEW_RATE_HZ`): driving + position + velocity, freshest of State / optional Visualization (latest-wins by AGV timestamp), with `source` + `data_age`. Not the VDA5050 `visualization` message. |

#### ROS 2 topic-name sanitization for `<mfg>` and `<serial>`

VDA5050 allows `serialNumber` characters `A-Z a-z 0-9 _ . : -`, and places
no character restriction on `manufacturer`. ROS 2 topic name *segments*
are stricter — each must match `^[A-Za-z_][A-Za-z0-9_]*$`. When a vendor
identity contains a leading digit or any of `. : -` (or other non-
`[A-Za-z0-9_]` chars), master rewrites the **ROS 2 topic path only**.
MQTT subscriptions, the master's internal AGV cache, and service request
payloads always use the raw `(manufacturer, serial_number)` values.

Rule (applies to each `<mfg>` / `<serial>` segment independently):

1. Replace any character outside `[A-Za-z0-9_]` with `_`.
2. If the result then starts with a digit, prepend `_`.

| Raw (mfg / serial) | ROS 2 segment |
|---|---|
| `KION` | `KION` |
| `S001` | `S001` |
| `001` | `_001` |
| `KION-001` | `KION_001` |
| `agv.42` | `agv_42` |
| `3M` | `_3M` |

Example: an AGV with `manufacturer="KION"`, `serial_number="001"` is
published at `/vda5050_master/KION/_001/order_status`, while its MQTT
topic remains `uagv/v2/KION/001/order_status`. Distinct raw identities
that sanitize to the same ROS 2 segment (e.g. `KION-001` and
`KION_001`) are not detected at onboarding; deployments should choose
serials that remain unambiguous after sanitization.

Master logs a one-shot `INFO` message the first time each AGV's identity
is sanitized, so the mapping is visible in startup logs.

### Global topics (4)

| Topic | Direction | QoS | Purpose |
|---|---|---|---|
| `/<ns>/assign_order_request` | external → master | RELIABLE / VOLATILE | Wire-async order dispatch. Caller publishes `AssignOrderRequest` with caller-generated `assignment_id` UUID |
| `/<ns>/assignment_results` | master → external | RELIABLE / VOLATILE | Per-`assignment_id` outcome. Decision enum: `ACCEPTED` / `QUEUED` / `REJECTED_PREFLIGHT` / `REJECTED_POSTFLIGHT` |
| `/<ns>/fleet_roster` | external → master | **RELIABLE / TRANSIENT_LOCAL (latched)** | Declarative full-state roster. Master diffs against current onboarded set, calls `onboard_agv_batch` / `offboard_agv_batch` to converge |
| `/<ns>/master_connection` | master → external | **RELIABLE / TRANSIENT_LOCAL (latched)** | Master liveness + readiness signal. State enum: `STARTING` → `READY` → `DEGRADED` → `READY` → `SHUTTING_DOWN`. 30 s heartbeat republish so subscribers can detect crashes by heartbeat-absence. Carries `master_id` (hostname-pid fallback), `master_version`, `broker_connected`, `onboarded_agv_count` |

### Services (13)

**Order dispatch**
- `assign_order` — sync order dispatch. Handles both new orders and updates. Returns `AssignmentDecision` + diagnostic errors[]
- `assign_instant_actions` — sync instant-action dispatch with mode-aware allowlist

**Fleet membership** — both ship in single + batch variants. Both route through `master.onboard_agv_batch` / `offboard_agv_batch` internally
- `onboard_agv` — single-AGV onboard (`AGVOnboardSpec` request → status enum)
- `onboard_agv_batch` — batch (`AGVOnboardSpec[]` → partial-success `onboarded[]` + `failed[]`)
- `offboard_agv` — single-AGV offboard (`AGVKey` request → status enum)
- `offboard_agv_batch` — batch (`AGVKey[]` → `offboarded_count`)

**Synchronous queries (operator diagnostics)**
- `get_device_status` — coherent (single-mutex) snapshot of state + connection + factsheet for one AGV
- `get_order_status` — atomic State + lifecycle bundle
- `get_pose_view` — fused pose snapshot for one AGV (same view as the `pose_view` stream)
- `get_loaded_map` — currently-loaded topology map + per-AGV factsheet-alignment summary
- `get_master_broker_status` — master's own MQTT-broker connection state + disconnect history

**Operator recovery (mode cancellation)**
- `resume_mode_cancelled_queue` — prepend the captured pre-mode-flip queue back into the live queue
- `discard_mode_cancelled_queue` — drop the captured queue

## Features implemented

**Order dispatch (master → AGV)**
- Validator chain: schema → graph → traversability → pre-send (AGV_NOT_ONBOARDED, AGV_OFFLINE, AGV_NO_STATE_YET, AGV_MODE_NOT_AUTO, AGV_POSITION_NOT_INITIALIZED, STATE_UNKNOWN)
- Order stitching (horizon extension via `order_update_id`), lifecycle tracking, mismatch recovery
- Instant actions with mode-aware allowlist

**AGV state ingestion (AGV → master)**
- Per-AGV cached State / Connection / Factsheet / Visualization with incoming schema validation
- Connection + State heartbeat monitoring; operational-state demotion on timeout
- Last-will detection (`kill -9` → CONNECTION_BROKEN)
- Mode-cancelled queue: capture on AUTOMATIC → non-AUTO flip; resume / discard on operator action

**Fleet operations**
- Onboard / offboard (idempotent), single + batch variants on the same underlying batch API
- Loaded map + factsheet alignment validator (vehicle-vs-map structural diff)
- MQTT broker disconnect / reconnect tracking with observability snapshot

**Async dispatch**
- Wire-async order dispatch via the `assign_order_request` topic — non-blocking equivalent of the sync `assign_order` service
- Caller-generated `assignment_id` UUID correlates `AssignmentResult` outcomes with `OrderStatus` lifecycle for the same order

**Fleet roster + liveness**
- `FleetRoster` subscriber (latched, full-state declarative); master diffs + converges by batch onboard/offboard
- `MasterConnection` publisher (latched, 30 s heartbeat) signals master readiness + degradation
- `master_id` defaults to `hostname-pid` for multi-master deployments without manual config

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
colcon build --packages-select vda5050_master_ros2
source install/setup.bash
```

## Run

```bash
# Broker (one time per machine)
mosquitto -v   # or: sudo systemctl start mosquitto

# Master
ros2 run vda5050_master_ros2 example_master

# AGV-side stub (for end-to-end testing)
ros2 run vda5050_master_ros2 mock_client --serial S001 --mfg Manufacturer
# OR for interop testing
ros2 run vda5050_master_ros2 example_client

# FMS-side driver (for end-to-end testing)
ros2 run vda5050_master_ros2 mock_fms --scenario happy-path
```

## Running the examples

**Master only** — exercise the 10 ROS 2 services with raw `ros2 service call` (no AGV side).
```bash
ros2 run vda5050_master_ros2 example_master &
ros2 service call /vda5050_master/onboard_agv vda5050_master_ros2/srv/OnboardAGV "{agv: {manufacturer: 'Manufacturer', serial_number: 'S001'}}"
```
- Expected response: `status: 0` (SUCCESS). A second call returns `status: 1` (ALREADY_ONBOARDED).
- Try also: `get_loaded_map`, `get_master_broker_status`, `assign_order` (rejects pre-AGV with `decision: 2` AGV_OFFLINE).

**Master + `mock_client`** — full order lifecycle plus AGV-side fault injection (validator rejections, broker bounce, last-will on kill -9, multi-AGV).
```bash
ros2 run vda5050_master_ros2 example_master &                       # T1
ros2 run vda5050_master_ros2 mock_client --serial S001 --mfg Manufacturer    # T2
```
- Expected: T1 (master) logs `[CONN] ONLINE Manufacturer/S001` within ~1 s.
- From a 3rd terminal, dispatch a sample order:
  ```bash
  ORDER_DIR=$(ros2 pkg prefix vda5050_master_ros2)/share/vda5050_master_ros2/sample_data/orders
  ros2 service call /vda5050_master/onboard_agv vda5050_master_ros2/srv/OnboardAGV "{agv: {manufacturer: 'Manufacturer', serial_number: 'S001'}}"
  ros2 service call /vda5050_master/assign_order vda5050_master_ros2/srv/AssignOrder "$(cat $ORDER_DIR/happy_path.yaml)"
  ```
  Expected: `decision: 0` (ASSIGNED), master `[STATE]` log advances `last_node_id: N0 → N1`, `phase: COMPLETED`.
- Fault-injection variants live under `sample_data/orders/`: `schema_reject.yaml`, `traversability_reject_*.yaml`, `stitch_base.yaml` + `stitch_update.yaml`.

**Master + `example_client` + `mock_fms`** — cross-implementation interop with scripted scenarios (custom CLI harness; exit code 0 = PASS).
```bash
ros2 run vda5050_master_ros2 example_master &                # T1
ros2 run vda5050_master_ros2 example_client &                # T2
ros2 run vda5050_master_ros2 mock_fms --scenario happy-path  # T3
echo $?    # 0 = PASS
```
- Scenarios: `happy-path`, `stitch`, `broker-bounce`, `onboard-flood`.
- Each step prints `OK: <step>` or `FAIL: <step>` to stdout; final exit code rolls them up.

## Sample data

Under `sample_data/`:

```
sample_map.json                            # 4-node grid loaded by example_master
orders/                                    # YAML orders for `ros2 service call`
  happy_path.yaml
  stitch_base.yaml, stitch_update.yaml
  schema_reject.yaml
  traversability_reject_*.yaml
instant_actions/                           # YAML instant actions
  state_request.yaml
  factsheet_request.yaml
  custom_beep.yaml
```

Installed to `share/vda5050_master_ros2/sample_data/` for runtime use:

```bash
ORDER_DIR=$(ros2 pkg prefix vda5050_master_ros2)/share/vda5050_master_ros2/sample_data/orders
ros2 service call /vda5050_master/assign_order vda5050_master_ros2/srv/AssignOrder "$(cat $ORDER_DIR/happy_path.yaml)"
```
