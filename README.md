# VDA5050 Library and Support Tools

`vda5050_core` is a modern C++ library for developing applications that communicate using the VDA5050 specification. It provides reusable components for both AGV-side and master-control implementations, including message types along with serialization and deserialization utilities, validation, execution utilities, MQTT communication and a high-level adapter API for robot integration.

The library is runtime framework-independent and can be integrated into standalone C++ applications, ROS 2 systems or existing robot software. It uses `ament_cmake` as its build system.

```
VDA5050 Master
      │ MQTT
      ▼
vda5050_core AGV Client
      │ navigation, action and state callbacks
      ▼
Robot SDK / REST API / ROS 2 Integration
      │
      ▼
    Robot
```

> **Status:** This project is under active development.



## Features

- **VDA5050 message types** represented as plain C++ structs.
- **JSON serialization and deserialization** for native types and optional ROS 2 `vda5050_interfaces` messages.
- **Validation** of orders, instant actions, protocol limits, action conflicts, factsheet alignment and graph traversability.
- **MQTT transport** built on Eclipse Paho.
- **An execution framework** for composing reactive, non-blocking robot logic.
- **A high-level AGV client adapter** for navigation, actions and state reporting.
- **An experimental master API** for fleet control: onboarding, order validation and stitching, lifecycle tracking and heartbeat monitoring.
- **Layout Interchange Format support** for loading and validating facility graphs.
- **Python bindings**, including helpers for migrating Open-RMF fleet adapters.



## Documentation


| Document                                                    | Contents                                    |
| ----------------------------------------------------------- | ------------------------------------------- |
| [Client Adapter Guide](vda5050_core/docs/client-adapter.md) | Integrating an AGV using the client adapter |
| [RMF Migration Guide](vda5050_core/docs/rmf-migration.md)   | Migrating an Open-RMF fleet adapter         |
| [Master Guide](vda5050_core/docs/master.md)                 | Building and evaluating the master API      |
| [Master API Reference](vda5050_core/docs/master-api.md)     | Every master command, type and callback     |
| [Validation Guide](vda5050_core/docs/validation.md)         | Specification checks and reading results    |
| [Design Guide](vda5050_core/docs/design.md)                 | Architecture and design rationale           |
| [Execution Guide](vda5050_core/docs/execution.md)           | Building custom execution logic             |
| [Types Guide](vda5050_core/docs/types.md)                   | Message types and JSON conversion           |


To connect an existing robot SDK, REST API or ROS 2 navigation system, start with the [Client Adapter Guide](vda5050_core/docs/client-adapter.md).

To build the fleet-control side, start with the [Master Guide](vda5050_core/docs/master.md).

To understand or extend the library architecture, start with the [Design Guide](vda5050_core/docs/design.md).

## Getting Started



### Requirements

- C++17
- CMake 3.8 or newer
- Eclipse Paho MQTT C++
- `nlohmann/json`
- `fmt`
- `pybind11` when building the Python bindings
- `vda5050_interfaces` when `ENABLE_ROS2=ON`



### Build

Install the required MQTT dependencies:

```
sudo apt update
sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev
```

Create a workspace, clone the repository and build the package:

```
mkdir -p ~/vda5050_ws/src
cd ~/vda5050_ws/src

git clone https://github.com/ros-industrial/vda5050_core.git

cd ~/vda5050_ws
colcon build --packages-select vda5050_core
source install/setup.bash
```



#### Build Options


| Option           | Default | Effect                                                  |
| ---------------- | ------- | ------------------------------------------------------- |
| `ENABLE_ROS2`    | `OFF`   | Enables support for ROS 2 `vda5050_interfaces` messages |
| `BUILD_PYTHON`   | `ON`    | Builds the Python bindings                              |
| `BUILD_EXAMPLES` | `ON`    | Builds the examples                                     |
| `BUILD_TESTING`  | `ON`    | Builds the tests and configured linters                 |




### Basic Usage

The following example shows the basic setup for an AGV-side client.

It creates an MQTT transport and a VDA5050 client adapter, then registers a navigation callback. In a real application, the callback should forward the request to the robot's navigation system.

```cpp
#include <iostream>

#include "vda5050_core/client/adapter/adapter.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

using namespace vda5050_core;

int main()
{
  auto mqtt_client = transport::create_default_client_unique(
    "tcp://localhost:1883",
    "agv_1");

  auto protocol_adapter = execution::ProtocolAdapter::make(
    std::move(mqtt_client),
    "uagv",
    "2.0.0",
    "Manufacturer",
    "S001");

  auto adapter = client::adapter::Adapter::make(protocol_adapter);

  adapter->on_navigate(
    [](auto node_request, auto edge_request, auto execution)
    {
      // Forward the request to the robot navigation system.
      //
      // This demonstration reports completion immediately.
      // A real integration should only report completion after
      // the robot reaches the requested node.
      execution->finished();
    });

  adapter->start();

  // Keep processing orders until Enter is pressed.
  std::cin.get();

  adapter->stop();
  return 0;
}
```

CMake integration:

```cmake
find_package(vda5050_core REQUIRED)

target_link_libraries(
  my_agv
  PRIVATE
  vda5050_core::client
)
```

For a complete integration covering navigation, actions, localization, cancellation and state reporting, see the [Client Adapter Guide](vda5050_core/docs/client-adapter.md) and [vda5050_core/examples/client/adapter_example.cpp](vda5050_core/examples/client/adapter_example.cpp).

The following example shows the basic setup for a master. The master API is experimental.

It creates an MQTT transport and a master, builds a two-node order, and assigns it once the AGV reports itself online, localized and idle. A real application drives orders from its own task source and assigns the next one on completion.

```cpp
#include <iostream>
#include <string>

#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

using namespace vda5050_core;

types::Order make_order(const std::string& id)
{
  types::Node start;
  start.node_id = "N0";
  start.sequence_id = 0;
  start.released = true;

  types::Node goal;
  goal.node_id = "N1";
  goal.sequence_id = 2;
  goal.released = true;

  types::Edge edge;
  edge.edge_id = "E0";
  edge.sequence_id = 1;
  edge.start_node_id = "N0";
  edge.end_node_id = "N1";
  edge.released = true;

  types::Order order;
  order.order_id = id;
  order.order_update_id = 0;
  order.nodes = {start, goal};
  order.edges = {edge};
  return order;
}

int main()
{
  auto mqtt_client = transport::create_default_client_shared(
    "tcp://localhost:1883",
    "master_1");

  auto master = master::VDA5050Master::make(mqtt_client);

  bool order_sent = false;

  // Assign the first order once the AGV is online, localized and idle.
  master->on_state(
    [&](const std::string& agv_id, const types::State& state)
    {
      if (order_sent) return;

      auto agv = master->get_agv("Manufacturer", "S001");
      if (!agv ||
          agv->get_operational_state() != master::AGVState::AVAILABLE)
      {
        return;
      }

      auto result = master->assign_order(
        "Manufacturer", "S001", make_order("order-1"));

      order_sent = result.decision ==
                   master::OrderAssignmentDecision::ASSIGNED;

      // If it was not assigned, result.errors says why.
    });

  master->on_order_complete(
    [](const std::string& agv_id, const std::string& order_id)
    {
      // Assign this AGV's next order.
    });

  master->connect();
  master->onboard_agv("uagv", "Manufacturer", "S001");

  std::cin.get();

  master->disconnect();
  return 0;
}
```

CMake integration:

```cmake
find_package(vda5050_core REQUIRED)

target_link_libraries(
  my_master
  PRIVATE
  vda5050_core::master
  vda5050_core::transport
)
```

For a complete integration covering order construction, validation, event handling and multi-AGV dispatch, see the [Master Guide](vda5050_core/docs/master.md) and [vda5050_core/examples/master/master_example.cpp](vda5050_core/examples/master/master_example.cpp).

## Examples

The following examples can be run against a local MQTT broker:

```bash
mosquitto -v
```


| Example                                                   | Demonstrates                              |
| --------------------------------------------------------- | ----------------------------------------- |
| `vda5050_core/examples/client/adapter_example.cpp`        | AGV client-adapter integration            |
| `vda5050_core/examples/master/order_publisher.cpp`        | Dispatching a VDA5050 order               |
| `vda5050_core/examples/master/master_example.cpp`         | A complete VDA5050 master implementation  |
| `vda5050_core/examples/execution/handler_integration.cpp` | Context, strategy and handler integration |
| `vda5050_core/examples/execution/engine_example.cpp`      | Event queues and wait conditions          |
| `vda5050_core/examples/execution/provider_example.cpp`    | Update broadcasting                       |
| `vda5050_core/examples/execution/custom_base.cpp`         | Defining custom updates and events        |




## Repository Structure

```
vda5050_core/
  include/vda5050_core/
    types/        VDA5050 message structs
    json_utils/   JSON serialization and traits
    validation/   Specification compliance checks
    errors/       Error codes and factories
    transport/    MQTT client interface and implementation
    execution/    Reactive execution framework
    client/       AGV-side client and adapter
    master/       Master-control components
    layout/       Layout Interchange Format support
    logger/       Logging
  examples/       Runnable examples
  python/         Python bindings
  test/           Unit and integration tests
  docs/           Documentation
```



## Testing

Run the test suite with:

```bash
colcon test --packages-select vda5050_core
colcon test-result --verbose
```

Some integration tests require an MQTT broker running on `localhost:1883`.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development and contribution guidelines.

Commits must include a `Signed-off-by` line certifying the [Developer Certificate of Origin](https://developercertificate.org/).

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.