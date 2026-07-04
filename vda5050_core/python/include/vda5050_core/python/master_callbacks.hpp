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

#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/paho_mqtt_client.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/visualization.hpp"

namespace vda5050_core {

namespace python {

namespace master {

/// \brief Python-facing adapter over VDA5050Master.
///
/// VDA5050Master reports events through virtual overrides. This subclass turns
/// each into a settable std::function callback (`master.on_state = fn`). The
/// virtuals fire on per-AGV MQTT threads, so every dispatch acquires the GIL
/// and swallows a raised Python exception so it can't abort that thread.
class PyMaster : public vda5050_core::master::VDA5050Master
{
public:
  using StateCb =
    std::function<void(const std::string&, const vda5050_core::types::State&)>;
  using ConnectionCb = std::function<void(
    const std::string&, const vda5050_core::types::Connection&)>;
  using VisualizationCb = std::function<void(
    const std::string&, const vda5050_core::types::Visualization&)>;
  using AgvIdCb = std::function<void(const std::string&)>;
  using NodeReachedCb =
    std::function<void(const std::string&, const std::string&)>;
  using ErrorsCb = std::function<void(
    const std::string&, const std::vector<vda5050_core::types::Error>&)>;
  using ModeChangedCb = std::function<void(
    const std::string&, vda5050_core::types::OperatingMode,
    vda5050_core::types::OperatingMode)>;
  using BoolFlagCb = std::function<void(const std::string&, bool)>;
  using BrokerCb = std::function<void()>;

  explicit PyMaster(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client)
  : VDA5050Master(std::move(mqtt_client))
  {
  }

  /// \brief Build a master with an internally-created Paho MQTT client.
  ///
  /// \param broker_address MQTT broker address (e.g. "tcp://localhost:1883").
  /// \param client_id      MQTT client id for this master.
  /// \return shared_ptr — the master requires make_shared
  ///         (enable_shared_from_this); a stack/unique instance breaks its
  ///         callback dispatch.
  static std::shared_ptr<PyMaster> create(
    const std::string& broker_address, const std::string& client_id)
  {
    auto mqtt_client = vda5050_core::transport::PahoMqttClient::make_shared(
      broker_address, client_id);
    return std::make_shared<PyMaster>(std::move(mqtt_client));
  }

  StateCb on_state_cb;
  ConnectionCb on_connection_cb;
  VisualizationCb on_visualization_cb;
  NodeReachedCb on_node_reached_cb;
  ErrorsCb on_errors_appeared_cb;
  ErrorsCb on_errors_resolved_cb;
  AgvIdCb on_new_base_requested_cb;
  ModeChangedCb on_mode_changed_cb;
  BoolFlagCb on_paused_cb;
  BoolFlagCb on_driving_cb;
  AgvIdCb on_connect_cb;
  AgvIdCb on_offline_cb;
  AgvIdCb on_connection_broken_cb;
  AgvIdCb on_state_timeout_cb;
  AgvIdCb on_state_resumed_cb;
  BrokerCb on_broker_disconnected_cb;
  BrokerCb on_broker_reconnected_cb;

  void on_state(
    const std::string& agv_id, const vda5050_core::types::State& state) override
  {
    dispatch(on_state_cb, "on_state", agv_id, state);
  }

  void on_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection) override
  {
    dispatch(on_connection_cb, "on_connection", agv_id, connection);
  }

  void on_visualization(
    const std::string& agv_id,
    const vda5050_core::types::Visualization& visualization) override
  {
    dispatch(on_visualization_cb, "on_visualization", agv_id, visualization);
  }

  void on_node_reached(
    const std::string& agv_id, const std::string& node_id) override
  {
    dispatch(on_node_reached_cb, "on_node_reached", agv_id, node_id);
  }

  void on_errors_appeared(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Error>& new_errors) override
  {
    dispatch(on_errors_appeared_cb, "on_errors_appeared", agv_id, new_errors);
  }

  void on_errors_resolved(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Error>& resolved_errors) override
  {
    dispatch(
      on_errors_resolved_cb, "on_errors_resolved", agv_id, resolved_errors);
  }

  void on_new_base_requested(const std::string& agv_id) override
  {
    dispatch(on_new_base_requested_cb, "on_new_base_requested", agv_id);
  }

  void on_mode_changed(
    const std::string& agv_id, vda5050_core::types::OperatingMode new_mode,
    vda5050_core::types::OperatingMode prev_mode) override
  {
    dispatch(
      on_mode_changed_cb, "on_mode_changed", agv_id, new_mode, prev_mode);
  }

  void on_paused(const std::string& agv_id, bool paused) override
  {
    dispatch(on_paused_cb, "on_paused", agv_id, paused);
  }

  void on_driving(const std::string& agv_id, bool driving) override
  {
    dispatch(on_driving_cb, "on_driving", agv_id, driving);
  }

  void on_connect(const std::string& agv_id) override
  {
    dispatch(on_connect_cb, "on_connect", agv_id);
  }

  void on_offline(const std::string& agv_id) override
  {
    dispatch(on_offline_cb, "on_offline", agv_id);
  }

  void on_connection_broken(const std::string& agv_id) override
  {
    dispatch(on_connection_broken_cb, "on_connection_broken", agv_id);
  }

  void on_state_timeout(const std::string& agv_id) override
  {
    dispatch(on_state_timeout_cb, "on_state_timeout", agv_id);
  }

  void on_state_resumed(const std::string& agv_id) override
  {
    dispatch(on_state_resumed_cb, "on_state_resumed", agv_id);
  }

  void on_broker_disconnected() override
  {
    dispatch(on_broker_disconnected_cb, "on_broker_disconnected");
  }

  void on_broker_reconnected() override
  {
    dispatch(on_broker_reconnected_cb, "on_broker_reconnected");
  }

private:
  /// \brief Run a Python callback under the GIL; unset → no-op, raised Python
  ///        exception is logged, not propagated onto the MQTT thread.
  template <typename Cb, typename... Args>
  void dispatch(const Cb& cb, const char* name, Args&&... args)
  {
    if (!cb) return;
    pybind11::gil_scoped_acquire gil;
    try
    {
      cb(std::forward<Args>(args)...);
    }
    catch (pybind11::error_already_set& e)
    {
      VDA5050_ERROR("Python {} callback raised: {}", name, e.what());
    }
    catch (const std::exception& e)
    {
      VDA5050_ERROR("{} callback error: {}", name, e.what());
    }
  }
};

}  // namespace master
}  // namespace python
}  // namespace vda5050_core

#endif  // VDA5050_CORE__PYTHON__MASTER_CALLBACKS_HPP_
