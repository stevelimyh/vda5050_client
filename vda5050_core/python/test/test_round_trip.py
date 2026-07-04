# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Round-trip test for the VDA5050 master Python bindings.

Injects a conforming Connection + State into the master through the AGV's
handle_* message hooks (exactly as the C++ integration tests do), then asserts
the Python callbacks fire with the real messages, the AGV becomes AVAILABLE,
its fused pose reflects the state, and assign_order then succeeds. This
exercises the full inbound path (message -> event detection -> callback
dispatch -> marshal into Python) plus the assign_order readiness gate. No
broker is required; messages are injected directly.
"""

import vda5050_master_python as v


def make_online_connection():
    c = v.types.Connection()
    c.header.manufacturer = "MFG"
    c.header.serial_number = "agv1"
    c.header.version = "2.0.0"
    c.connection_state = v.types.ConnectionState.ONLINE
    return c


def make_ready_state():
    s = v.types.State()
    s.header.manufacturer = "MFG"
    s.header.serial_number = "agv1"
    s.header.version = "2.0.0"
    s.order_id = ""
    s.order_update_id = 0
    s.last_node_id = "N0"
    s.operating_mode = v.types.OperatingMode.AUTOMATIC
    pos = v.types.AGVPosition()
    pos.position_initialized = True
    pos.map_id = "test_map"
    s.agv_position = pos
    return s


def main():
    master = v.master.Master("tcp://localhost:1883", "roundtrip-fms")
    master.onboard_agv("MFG", "agv1")
    agv = master.get_agv("MFG", "agv1")

    fired = {}
    master.on_connection = lambda aid, c: fired.__setitem__(
        "on_connection", c.connection_state)
    master.on_state = lambda aid, s: fired.__setitem__(
        "on_state", s.operating_mode)

    # Inject the AGV's messages, as the C++ integration tests do.
    agv.handle_connection(make_online_connection())
    agv.handle_state(make_ready_state())

    # 1. Callbacks fired with the real messages (the marshaling round-trip).
    assert fired.get("on_connection") == v.types.ConnectionState.ONLINE, fired
    assert fired.get("on_state") == v.types.OperatingMode.AUTOMATIC, fired

    # 2. Event detection: the AGV is now AVAILABLE.
    assert agv.get_operational_state() == v.master.AGVState.AVAILABLE, (
        agv.get_operational_state())

    # 3. The fused pose reflects the injected state.
    pose = agv.get_pose_view()
    assert pose.source == v.master.PoseSource.STATE, pose.source
    assert pose.agv_position.map_id == "test_map"

    # 4. The AGV is now assignable — the full usable gate.
    order = v.types.Order()
    order.header.manufacturer = "MFG"
    order.header.serial_number = "agv1"
    order.header.version = "2.0.0"
    order.order_id = "rt_order"
    node = v.types.Node()
    node.node_id = "N0"
    node.sequence_id = 0
    node.released = True
    order.nodes = [node]
    result = master.assign_order("MFG", "agv1", order)
    assert result.decision == v.master.AssignmentDecision.ASSIGNED, (
        result.decision)
    assert bool(result)

    print("ROUND-TRIP PASSED")
    print("  callbacks fired:", {k: str(x) for k, x in fired.items()})
    print("  operational_state:", agv.get_operational_state())
    print("  pose_view source:", pose.source, "| map:", pose.agv_position.map_id)
    print("  assign_order:", result.decision)


if __name__ == "__main__":
    main()
