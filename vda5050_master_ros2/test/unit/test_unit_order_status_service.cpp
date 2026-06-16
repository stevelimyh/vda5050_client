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
#include "vda5050_master_ros2/order_status_service.hpp"
#include "vda5050_master_ros2/srv/get_order_status.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using GetOrderStatus = vda5050_master_ros2::srv::GetOrderStatus;
using OrderStatus = vda5050_master_ros2::msg::OrderStatus;

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
  const std::string& mfg, const std::string& serial,
  const std::string& order_id = "ORD1", uint32_t last_node_seq = 1)
{
  vda5050_core::types::State s;
  s.header.header_id = 1;
  s.header.timestamp = std::chrono::system_clock::now();
  s.header.version = "2.0.0";
  s.header.manufacturer = mfg;
  s.header.serial_number = serial;
  s.order_id = order_id;
  s.order_update_id = 0;
  s.last_node_id = last_node_seq > 0 ? "N1" : "";
  s.last_node_sequence_id = last_node_seq;
  s.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  return s;
}

class OrderStatusServiceTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

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
    node_ = rclcpp::Node::make_shared("order_status_service_test_node");
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

  OrderStatusService::AgvLookup make_lookup()
  {
    return [this](const std::string& mfg, const std::string& serial)
             -> std::shared_ptr<vda5050_core::master::AGV> {
      auto it = agvs_.find(mfg + "/" + serial);
      if (it == agvs_.end()) return nullptr;
      return it->second;
    };
  }

  std::shared_ptr<GetOrderStatus::Response> call(
    const std::string& service_name, const std::string& mfg,
    const std::string& serial,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<GetOrderStatus>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<GetOrderStatus::Request>();
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

TEST_F(OrderStatusServiceTest, ServiceNameFollowsConvention)
{
  OrderStatusService svc(node_, make_lookup());
  EXPECT_EQ(svc.service_name(), "/vda5050_master/get_order_status");
}

TEST_F(OrderStatusServiceTest, CustomNamespaceAppliesToServiceName)
{
  OrderStatusService svc(node_, make_lookup(), nullptr, "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/get_order_status");
}

// =============================================================================
// Status branches
// =============================================================================

TEST_F(OrderStatusServiceTest, ReturnsSuccessWhenAgvOnboardedWithState)
{
  // Phase derivation is exhaustively tested in test_order_status_builder;
  // here we only verify the service callback's plumbing: SUCCESS branch
  // populates the order_status payload and state_received_at timestamp,
  // and echoes mfg/serial. Without a tracked active order the phase is
  // NO_ORDER but the order_id field still echoes the vda5050_core::master::AGV's current
  // State.order_id (per builder fallback semantics).
  auto agv = add_agv(kMfg, kSerial);
  agv->handle_state(make_state(kMfg, kSerial, "ORD1", /*last_node_seq=*/1));

  OrderStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, GetOrderStatus::Response::SUCCESS);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
  EXPECT_EQ(resp->order_status.manufacturer, kMfg);
  EXPECT_EQ(resp->order_status.serial_number, kSerial);
  EXPECT_EQ(resp->order_status.order_id, "ORD1");
  EXPECT_EQ(resp->order_status.last_node_id, "N1");
  EXPECT_EQ(resp->order_status.last_node_sequence_id, 1u);
  ASSERT_EQ(resp->state_received_at.size(), 1u);
  EXPECT_GT(resp->state_received_at[0].sec, 0);
}

TEST_F(OrderStatusServiceTest, ReturnsAgvNotOnboardedForUnknownAgv)
{
  OrderStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, GetOrderStatus::Response::AGV_NOT_ONBOARDED);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
}

TEST_F(OrderStatusServiceTest, ReturnsAgvSilentWhenNoStateAndNoActiveOrder)
{
  // Onboarded but no state and no order tracked.
  add_agv(kMfg, kSerial);

  OrderStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, GetOrderStatus::Response::AGV_SILENT);
}

TEST_F(OrderStatusServiceTest, ReturnsInvalidRequestForEmptyMfgOrSerial)
{
  OrderStatusService svc(node_, make_lookup());

  auto r1 = call(svc.service_name(), "", kSerial);
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->status, GetOrderStatus::Response::INVALID_REQUEST);

  auto r2 = call(svc.service_name(), kMfg, "");
  ASSERT_NE(r2, nullptr);
  EXPECT_EQ(r2->status, GetOrderStatus::Response::INVALID_REQUEST);
}

TEST_F(OrderStatusServiceTest, SuccessWithPhaseNoOrderWhenStateButNoOrder)
{
  // vda5050_core::master::AGV reported State (so not SILENT) but no active order tracked.
  // Spec-conformant case: vda5050_core::master::AGV onboarded, idle, no order.
  auto agv = add_agv(kMfg, kSerial);
  agv->handle_state(
    make_state(kMfg, kSerial, /*order_id=*/"", /*last_node_seq=*/0));

  OrderStatusService svc(node_, make_lookup());
  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, GetOrderStatus::Response::SUCCESS);
  EXPECT_EQ(resp->order_status.phase, OrderStatus::PHASE_NO_ORDER);
}

// =============================================================================
// Multi-vda5050_core::master::AGV
// =============================================================================

TEST_F(OrderStatusServiceTest, MultiAgvIsolation)
{
  auto agv_a = add_agv("ACME", "AGV01");
  agv_a->handle_state(make_state("ACME", "AGV01", "ORD-A", 1));

  auto agv_b = add_agv("OTHER", "AGV02");
  agv_b->handle_state(make_state("OTHER", "AGV02", "ORD-B", 1));

  OrderStatusService svc(node_, make_lookup());

  auto r_a = call(svc.service_name(), "ACME", "AGV01");
  ASSERT_NE(r_a, nullptr);
  EXPECT_EQ(r_a->status, GetOrderStatus::Response::SUCCESS);
  EXPECT_EQ(r_a->order_status.order_id, "ORD-A");
  EXPECT_EQ(r_a->order_status.serial_number, "AGV01");

  auto r_b = call(svc.service_name(), "OTHER", "AGV02");
  ASSERT_NE(r_b, nullptr);
  EXPECT_EQ(r_b->status, GetOrderStatus::Response::SUCCESS);
  EXPECT_EQ(r_b->order_status.order_id, "ORD-B");
  EXPECT_EQ(r_b->order_status.serial_number, "AGV02");
}

}  // namespace test
}  // namespace vda5050_master_ros2
