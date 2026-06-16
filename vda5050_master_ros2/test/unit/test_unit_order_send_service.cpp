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
#include <utility>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/assignment_result.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_interfaces/msg/order.hpp"
#include "vda5050_master_ros2/order_send_service.hpp"
#include "vda5050_master_ros2/srv/assign_order.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using AssignOrder = vda5050_master_ros2::srv::AssignOrder;

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

vda5050_interfaces::msg::Order make_order_msg(
  const std::string& order_id, uint32_t order_update_id)
{
  vda5050_interfaces::msg::Order msg;
  msg.header.header_id = 1;
  msg.header.version = "2.0.0";
  msg.header.manufacturer = kMfg;
  msg.header.serial_number = kSerial;
  msg.order_id = order_id;
  msg.order_update_id = order_update_id;
  // Add one trivial node so order is structurally valid for from_msg
  // round-trip assertions.
  vda5050_interfaces::msg::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  msg.nodes.push_back(n);
  return msg;
}

vda5050_core::types::Error make_error(
  const std::string& error_type, vda5050_core::types::ErrorLevel level)
{
  vda5050_core::types::Error e;
  e.error_type = error_type;
  e.error_level = level;
  return e;
}

// Stubbed OrderSender: captures call args + canned-result control via
// shared state so tests can both read what the service handed off and
// dictate what it gets back.
struct SenderStub
{
  std::shared_ptr<std::atomic<int>> calls{
    std::make_shared<std::atomic<int>>(0)};
  std::shared_ptr<vda5050_core::types::Order> last_order{
    std::make_shared<vda5050_core::types::Order>()};
  std::shared_ptr<std::string> last_mfg{std::make_shared<std::string>()};
  std::shared_ptr<std::string> last_serial{std::make_shared<std::string>()};
  std::shared_ptr<vda5050_core::master::AssignmentResult> canned{
    std::make_shared<vda5050_core::master::AssignmentResult>()};

  OrderSendService::OrderSender as_callable()
  {
    auto c = calls;
    auto lo = last_order;
    auto lm = last_mfg;
    auto ls = last_serial;
    auto cn = canned;
    return [c, lo, lm, ls, cn](
             const std::string& mfg, const std::string& serial,
             const vda5050_core::types::Order& order)
             -> vda5050_core::master::AssignmentResult {
      c->fetch_add(1);
      *lm = mfg;
      *ls = serial;
      *lo = order;
      return *cn;
    };
  }
};

class OrderSendServiceTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("order_send_service_test_node");
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
  }

  std::shared_ptr<AssignOrder::Response> call(
    const std::string& service_name, const std::string& mfg,
    const std::string& serial, const vda5050_interfaces::msg::Order& order,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<AssignOrder>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<AssignOrder::Request>();
    request->manufacturer = mfg;
    request->serial_number = serial;
    request->order = order;
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
// Service name + namespace
// =============================================================================

TEST_F(OrderSendServiceTest, ServiceNameFollowsConvention)
{
  SenderStub stub;
  OrderSendService svc(node_, stub.as_callable());
  EXPECT_EQ(svc.service_name(), "/vda5050_master/assign_order");
}

TEST_F(OrderSendServiceTest, CustomNamespaceAppliesToServiceName)
{
  SenderStub stub;
  OrderSendService svc(node_, stub.as_callable(), "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/assign_order");
}

// =============================================================================
// Decision branches
// =============================================================================

TEST_F(OrderSendServiceTest, ReturnsAssignedOnSuccess)
{
  SenderStub stub;
  stub.canned->decision = vda5050_core::master::AssignmentDecision::ASSIGNED;
  OrderSendService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial, make_order_msg("O1", 0));

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->decision, AssignOrder::Response::ASSIGNED);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
  EXPECT_EQ(resp->order_id, "O1");
  EXPECT_EQ(resp->order_update_id, 0u);
  EXPECT_TRUE(resp->errors.empty());
  EXPECT_EQ(stub.calls->load(), 1);
}

TEST_F(OrderSendServiceTest, ReturnsAgvNotOnboardedWhenSenderReports)
{
  SenderStub stub;
  stub.canned->decision =
    vda5050_core::master::AssignmentDecision::AGV_NOT_ONBOARDED;
  OrderSendService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial, make_order_msg("O1", 0));

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->decision, AssignOrder::Response::AGV_NOT_ONBOARDED);
  EXPECT_TRUE(resp->errors.empty());
}

TEST_F(OrderSendServiceTest, ReturnsInvalidRequestForEmptyMfgOrSerial)
{
  SenderStub stub;
  stub.canned->decision = vda5050_core::master::AssignmentDecision::ASSIGNED;
  OrderSendService svc(node_, stub.as_callable());

  auto r1 = call(svc.service_name(), "", kSerial, make_order_msg("O1", 0));
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->decision, AssignOrder::Response::INVALID_REQUEST);

  auto r2 = call(svc.service_name(), kMfg, "", make_order_msg("O1", 0));
  ASSERT_NE(r2, nullptr);
  EXPECT_EQ(r2->decision, AssignOrder::Response::INVALID_REQUEST);

  // Sender NOT invoked for either.
  EXPECT_EQ(stub.calls->load(), 0);
}

TEST_F(OrderSendServiceTest, ErrorsAreForwardedOnRejection)
{
  SenderStub stub;
  stub.canned->decision =
    vda5050_core::master::AssignmentDecision::STITCH_REJECTED;
  stub.canned->errors = {
    make_error("stitchPointPassed", vda5050_core::types::ErrorLevel::WARNING),
    make_error(
      "orderUpdateIdTooLow", vda5050_core::types::ErrorLevel::WARNING)};
  OrderSendService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial, make_order_msg("O1", 1));

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->decision, AssignOrder::Response::STITCH_REJECTED);
  ASSERT_EQ(resp->errors.size(), 2u);
  EXPECT_EQ(resp->errors[0].error_type, "stitchPointPassed");
  EXPECT_EQ(resp->errors[1].error_type, "orderUpdateIdTooLow");
}

// =============================================================================
// Round-trip: from_msg<>
// =============================================================================

TEST_F(OrderSendServiceTest, OrderIsConvertedAndForwardedToSender)
{
  SenderStub stub;
  stub.canned->decision = vda5050_core::master::AssignmentDecision::ASSIGNED;
  OrderSendService svc(node_, stub.as_callable());

  auto order_msg = make_order_msg("ORD-RT", 5);
  auto resp = call(svc.service_name(), kMfg, kSerial, order_msg);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->decision, AssignOrder::Response::ASSIGNED);
  EXPECT_EQ(*stub.last_mfg, kMfg);
  EXPECT_EQ(*stub.last_serial, kSerial);
  EXPECT_EQ(stub.last_order->order_id, "ORD-RT");
  EXPECT_EQ(stub.last_order->order_update_id, 5u);
  EXPECT_EQ(stub.last_order->nodes.size(), order_msg.nodes.size());
  EXPECT_EQ(stub.last_order->nodes.front().node_id, "N0");
}

// =============================================================================
// Defense against future enum drift
// =============================================================================

TEST_F(OrderSendServiceTest, MapsAllAssignmentDecisions)
{
  struct Case
  {
    vda5050_core::master::AssignmentDecision in;
    uint8_t expected;
  };
  const std::vector<Case> cases = {
    {vda5050_core::master::AssignmentDecision::ASSIGNED,
     AssignOrder::Response::ASSIGNED},
    {vda5050_core::master::AssignmentDecision::AGV_NOT_ONBOARDED,
     AssignOrder::Response::AGV_NOT_ONBOARDED},
    {vda5050_core::master::AssignmentDecision::AGV_OFFLINE,
     AssignOrder::Response::AGV_OFFLINE},
    {vda5050_core::master::AssignmentDecision::AGV_NOT_READY,
     AssignOrder::Response::AGV_NOT_READY},
    {vda5050_core::master::AssignmentDecision::AGV_MODE_NOT_AUTO,
     AssignOrder::Response::AGV_MODE_NOT_AUTO},
    {vda5050_core::master::AssignmentDecision::AGV_POSITION_NOT_INITIALIZED,
     AssignOrder::Response::AGV_POSITION_NOT_INITIALIZED},
    {vda5050_core::master::AssignmentDecision::AGV_NO_STATE_YET,
     AssignOrder::Response::AGV_NO_STATE_YET},
    {vda5050_core::master::AssignmentDecision::STITCH_REJECTED,
     AssignOrder::Response::STITCH_REJECTED},
    {vda5050_core::master::AssignmentDecision::STITCH_QUEUED,
     AssignOrder::Response::STITCH_QUEUED}};

  for (const auto& c : cases)
  {
    SenderStub stub;
    stub.canned->decision = c.in;
    OrderSendService svc(node_, stub.as_callable());
    auto resp = call(svc.service_name(), kMfg, kSerial, make_order_msg("O", 0));
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->decision, c.expected)
      << "Mismatch for vda5050_core::master::AssignmentDecision="
      << static_cast<int>(c.in);
  }
}

}  // namespace test
}  // namespace vda5050_master_ros2
