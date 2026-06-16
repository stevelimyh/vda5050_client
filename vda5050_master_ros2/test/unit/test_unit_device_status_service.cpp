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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_master_ros2/device_status_service.hpp"
#include "vda5050_master_ros2/srv/get_device_status.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";
constexpr const char* kOtherMfg = "OTHER";
constexpr const char* kOtherSerial = "AGV02";

using GetDeviceStatus = vda5050_master_ros2::srv::GetDeviceStatus;

// Poll a predicate until true or timeout. Returns true if predicate passed.
template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

vda5050_core::types::State make_state(
  const std::string& mfg, const std::string& serial)
{
  vda5050_core::types::State state;
  state.header.header_id = 1;
  state.header.timestamp = std::chrono::system_clock::now();
  state.header.version = "2.0.0";
  state.header.manufacturer = mfg;
  state.header.serial_number = serial;
  state.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  return state;
}

vda5050_core::types::Connection make_connection(
  const std::string& mfg, const std::string& serial)
{
  vda5050_core::types::Connection conn;
  conn.header.header_id = 1;
  conn.header.timestamp = std::chrono::system_clock::now();
  conn.header.version = "2.0.0";
  conn.header.manufacturer = mfg;
  conn.header.serial_number = serial;
  conn.connection_state = vda5050_core::types::ConnectionState::ONLINE;
  return conn;
}

vda5050_core::types::Factsheet make_factsheet(
  const std::string& mfg, const std::string& serial)
{
  vda5050_core::types::Factsheet fs;
  fs.header.header_id = 1;
  fs.header.timestamp = std::chrono::system_clock::now();
  fs.header.version = "2.0.0";
  fs.header.manufacturer = mfg;
  fs.header.serial_number = serial;
  return fs;
}

class DeviceStatusServiceTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  // Lookup-backing map: callers populate this; the lookup lambda
  // searches by "{mfg}/{serial}" key.
  std::unordered_map<std::string, std::shared_ptr<vda5050_core::master::AGV>>
    agvs_;

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("device_status_service_test_node");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_ = true;
    spin_thread_ = std::thread([this] {
      while (running_ && rclcpp::ok())
      {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });
  }

  void TearDown() override
  {
    running_ = false;
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.reset();
    node_.reset();
    agvs_.clear();
  }

  std::shared_ptr<vda5050_core::master::AGV> add_agv(
    const std::string& mfg, const std::string& serial)
  {
    auto agv =
      std::make_shared<vda5050_core::master::AGV>(nullptr, mfg, serial);
    agvs_[mfg + "/" + serial] = agv;
    return agv;
  }

  DeviceStatusService::AgvLookup make_lookup()
  {
    return [this](const std::string& mfg, const std::string& serial)
             -> std::shared_ptr<vda5050_core::master::AGV> {
      auto it = agvs_.find(mfg + "/" + serial);
      if (it == agvs_.end()) return nullptr;
      return it->second;
    };
  }

  // Issue an async service call and block until response or timeout.
  // Returns nullptr on timeout. Spawns a transient client on the test
  // node so each call gets a fresh future.
  std::shared_ptr<GetDeviceStatus::Response> call(
    const std::string& service_name, const std::string& mfg,
    const std::string& serial,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<GetDeviceStatus>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<GetDeviceStatus::Request>();
    request->manufacturer = mfg;
    request->serial_number = serial;
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }
};

}  // namespace

// =============================================================================
// Service name + lifecycle
// =============================================================================

TEST_F(DeviceStatusServiceTest, ServiceNameFollowsConvention)
{
  DeviceStatusService svc(node_, make_lookup());
  EXPECT_EQ(svc.service_name(), "/vda5050_master/get_device_status");
}

TEST_F(DeviceStatusServiceTest, CustomNamespaceAppliesToServiceName)
{
  DeviceStatusService svc(node_, make_lookup(), "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/get_device_status");
}

// =============================================================================
// Status branches
// =============================================================================

TEST_F(DeviceStatusServiceTest, ReturnsSuccessWhenAgvOnboardedWithFullData)
{
  auto agv = add_agv(kMfg, kSerial);
  agv->handle_state(make_state(kMfg, kSerial));
  agv->handle_connection(make_connection(kMfg, kSerial));
  agv->handle_factsheet(make_factsheet(kMfg, kSerial));

  DeviceStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, GetDeviceStatus::Response::SUCCESS);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
  ASSERT_EQ(resp->state.size(), 1u);
  ASSERT_EQ(resp->connection.size(), 1u);
  ASSERT_EQ(resp->factsheet.size(), 1u);
  EXPECT_EQ(resp->state[0].header.manufacturer, kMfg);
  EXPECT_EQ(resp->state[0].header.serial_number, kSerial);
  EXPECT_EQ(resp->connection[0].header.serial_number, kSerial);
  EXPECT_EQ(resp->factsheet[0].header.serial_number, kSerial);
  // Timestamps populated (sec part > 0 since we used system_clock).
  ASSERT_EQ(resp->state_received_at.size(), 1u);
  ASSERT_EQ(resp->connection_received_at.size(), 1u);
  ASSERT_EQ(resp->factsheet_received_at.size(), 1u);
  EXPECT_GT(resp->state_received_at[0].sec, 0);
  EXPECT_GT(resp->connection_received_at[0].sec, 0);
  EXPECT_GT(resp->factsheet_received_at[0].sec, 0);
}

TEST_F(DeviceStatusServiceTest, ReturnsAgvNotOnboardedForUnknownAgv)
{
  // No AGVs registered.
  DeviceStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, GetDeviceStatus::Response::AGV_NOT_ONBOARDED);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
  EXPECT_TRUE(resp->state.empty());
  EXPECT_TRUE(resp->connection.empty());
  EXPECT_TRUE(resp->factsheet.empty());
}

TEST_F(DeviceStatusServiceTest, ReturnsAgvSilentWhenCacheEmpty)
{
  // Onboarded but no handle_* calls — cache empty.
  add_agv(kMfg, kSerial);

  DeviceStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, GetDeviceStatus::Response::AGV_SILENT);
  EXPECT_TRUE(resp->state.empty());
  EXPECT_TRUE(resp->connection.empty());
  EXPECT_TRUE(resp->factsheet.empty());
}

TEST_F(DeviceStatusServiceTest, ReturnsInvalidRequestForEmptyMfgOrSerial)
{
  DeviceStatusService svc(node_, make_lookup());

  auto r1 = call(svc.service_name(), "", kSerial);
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->status, GetDeviceStatus::Response::INVALID_REQUEST);

  auto r2 = call(svc.service_name(), kMfg, "");
  ASSERT_NE(r2, nullptr);
  EXPECT_EQ(r2->status, GetDeviceStatus::Response::INVALID_REQUEST);

  auto r3 = call(svc.service_name(), "", "");
  ASSERT_NE(r3, nullptr);
  EXPECT_EQ(r3->status, GetDeviceStatus::Response::INVALID_REQUEST);
}

TEST_F(DeviceStatusServiceTest, ReturnsPartialDataWhenFactsheetMissing)
{
  // State + connection but never received a factsheet — common vda5050_core::master::AGV
  // bring-up scenario before the vda5050_core::master::AGV pushes its factsheet.
  auto agv = add_agv(kMfg, kSerial);
  agv->handle_state(make_state(kMfg, kSerial));
  agv->handle_connection(make_connection(kMfg, kSerial));

  DeviceStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, GetDeviceStatus::Response::SUCCESS);
  EXPECT_EQ(resp->state.size(), 1u);
  EXPECT_EQ(resp->connection.size(), 1u);
  EXPECT_TRUE(resp->factsheet.empty());
  // No factsheet yet -> bounded-array timestamp also empty.
  EXPECT_TRUE(resp->factsheet_received_at.empty());
}

// =============================================================================
// Multi-vda5050_core::master::AGV + concurrency
// =============================================================================

TEST_F(DeviceStatusServiceTest, MultiAgvIsolation)
{
  auto agv_a = add_agv(kMfg, kSerial);
  agv_a->handle_state(make_state(kMfg, kSerial));
  auto agv_b = add_agv(kOtherMfg, kOtherSerial);
  agv_b->handle_state(make_state(kOtherMfg, kOtherSerial));

  DeviceStatusService svc(node_, make_lookup());

  auto resp_a = call(svc.service_name(), kMfg, kSerial);
  ASSERT_NE(resp_a, nullptr);
  EXPECT_EQ(resp_a->status, GetDeviceStatus::Response::SUCCESS);
  ASSERT_EQ(resp_a->state.size(), 1u);
  EXPECT_EQ(resp_a->state[0].header.manufacturer, kMfg);
  EXPECT_EQ(resp_a->state[0].header.serial_number, kSerial);

  auto resp_b = call(svc.service_name(), kOtherMfg, kOtherSerial);
  ASSERT_NE(resp_b, nullptr);
  EXPECT_EQ(resp_b->status, GetDeviceStatus::Response::SUCCESS);
  ASSERT_EQ(resp_b->state.size(), 1u);
  EXPECT_EQ(resp_b->state[0].header.manufacturer, kOtherMfg);
  EXPECT_EQ(resp_b->state[0].header.serial_number, kOtherSerial);
}

TEST_F(DeviceStatusServiceTest, ConcurrentClientsReturnIndependentResponses)
{
  auto agv = add_agv(kMfg, kSerial);
  agv->handle_state(make_state(kMfg, kSerial));
  agv->handle_connection(make_connection(kMfg, kSerial));

  DeviceStatusService svc(node_, make_lookup());

  // Two concurrent service clients on the same node, same target vda5050_core::master::AGV.
  // Single-threaded executor still serializes the actual callback
  // dispatch — this is a smoke test that two pending requests resolve
  // cleanly, not a stress test.
  auto client = node_->create_client<GetDeviceStatus>(svc.service_name());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto request = std::make_shared<GetDeviceStatus::Request>();
  request->manufacturer = kMfg;
  request->serial_number = kSerial;

  auto fut1 = client->async_send_request(request);
  auto fut2 = client->async_send_request(request);

  ASSERT_EQ(fut1.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(fut2.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto r1 = fut1.get();
  auto r2 = fut2.get();
  EXPECT_EQ(r1->status, GetDeviceStatus::Response::SUCCESS);
  EXPECT_EQ(r2->status, GetDeviceStatus::Response::SUCCESS);
  EXPECT_EQ(r1->state.size(), 1u);
  EXPECT_EQ(r2->state.size(), 1u);
}

}  // namespace test
}  // namespace vda5050_master_ros2
