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

#ifndef VDA5050_CORE__PYTHON__MASTER_CALLBACKS_HPP_
#define VDA5050_CORE__PYTHON__MASTER_CALLBACKS_HPP_

#include <pybind11/pybind11.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/paho_mqtt_client.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/visualization.hpp"

namespace vda5050_core::python::master {

/// \brief Python-facing adapter over VDA5050Master.
///
/// VDA5050Master reports events through virtual overrides (on_state, ...).
/// Python can't subclass those cleanly, so this subclass turns each virtual
/// into a settable std::function callback (bound as master.on_state = fn).
/// Each override acquires the GIL before entering Python, since the virtuals
/// fire on the per-AGV MQTT threads, not the Python thread.
class PyMaster : public vda5050_core::master::VDA5050Master
{
public:
  using StateCb =
    std::function<void(std::string, vda5050_core::types::State)>;
  using ConnectionCb =
    std::function<void(std::string, vda5050_core::types::Connection)>;
  using FactsheetCb =
    std::function<void(std::string, vda5050_core::types::Factsheet)>;
  using VisualizationCb =
    std::function<void(std::string, vda5050_core::types::Visualization)>;

  explicit PyMaster(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client)
  : VDA5050Master(std::move(mqtt_client))
  {
  }

  /// \brief Convenience factory: build the Paho MQTT client internally.
  ///
  /// \param broker_address MQTT broker address (e.g. "tcp://localhost:1883").
  /// \param client_id      MQTT client id for this master.
  static std::shared_ptr<PyMaster> create(
    const std::string& broker_address, const std::string& client_id)
  {
    auto mqtt_client = vda5050_core::transport::PahoMqttClient::make_shared(
      broker_address, client_id);
    return std::make_shared<PyMaster>(std::move(mqtt_client));
  }

  StateCb on_state_cb;
  ConnectionCb on_connection_cb;
  FactsheetCb on_factsheet_cb;
  VisualizationCb on_visualization_cb;

  void on_state(
    const std::string& agv_id,
    const vda5050_core::types::State& state) override
  {
    if (on_state_cb)
    {
      pybind11::gil_scoped_acquire gil;
      on_state_cb(agv_id, state);
    }
  }

  void on_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection) override
  {
    if (on_connection_cb)
    {
      pybind11::gil_scoped_acquire gil;
      on_connection_cb(agv_id, connection);
    }
  }

  void on_factsheet(
    const std::string& agv_id,
    const vda5050_core::types::Factsheet& factsheet) override
  {
    if (on_factsheet_cb)
    {
      pybind11::gil_scoped_acquire gil;
      on_factsheet_cb(agv_id, factsheet);
    }
  }

  void on_visualization(
    const std::string& agv_id,
    const vda5050_core::types::Visualization& visualization) override
  {
    if (on_visualization_cb)
    {
      pybind11::gil_scoped_acquire gil;
      on_visualization_cb(agv_id, visualization);
    }
  }
};

}  // namespace vda5050_core::python::master

#endif  // VDA5050_CORE__PYTHON__MASTER_CALLBACKS_HPP_
