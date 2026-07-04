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

"""Minimal VDA5050 fleet-master example.

Connects to an MQTT broker, onboards an AGV, prints incoming state/connection,
and assigns a one-node order. Run against a running mosquitto broker:

    python3 master_example.py

Notes:
    - Do not capture ``master`` (or an AGV handle) inside a callback: it forms a
      reference cycle Python's GC cannot collect. Use the ``agv_id`` argument and
      ``master.get_agv(...)`` instead.
    - AGV handles from ``get_agv()`` are valid only while the master is alive.
"""

import time

import vda5050_master_python as v


def build_order():
    order = v.types.Order()
    order.order_id = "example_order"
    order.order_update_id = 0
    node = v.types.Node()
    node.node_id = "N0"
    node.sequence_id = 0
    node.released = True
    order.nodes = [node]          # assign the whole list (do not .append)
    return order


def main():
    master = v.master.Master("tcp://localhost:1883", "example-fms")

    # Fleet logic lives in Python: set the callbacks you care about.
    master.on_state = lambda agv_id, state: print(
        f"[state] {agv_id}: order={state.order_id} mode={state.operating_mode}")
    master.on_connection = lambda agv_id, conn: print(
        f"[connection] {agv_id}: {conn.connection_state}")

    master.connect()
    master.onboard_agv("MFG", "agv1")

    result = master.assign_order("MFG", "agv1", build_order())
    print(f"assign_order -> {result.decision} (ok={bool(result)})")

    try:
        time.sleep(2.0)           # spin so callbacks can fire
    finally:
        master.disconnect()       # explicit shutdown before the object is dropped


if __name__ == "__main__":
    main()
