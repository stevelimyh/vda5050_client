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
#include "vda5050_core/master/actions/instant_action_assignment_result.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_interfaces/msg/instant_actions.hpp"
#include "vda5050_master_ros2/instant_actions_send_service.hpp"
#include "vda5050_master_ros2/srv/assign_instant_actions.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using AssignInstantActions = vda5050_master_ros2::srv::AssignInstantActions;

vda5050_interfaces::msg::InstantActions make_actions_msg(
  const std::string& action_id, const std::string& action_type)
{
  vda5050_interfaces::msg::InstantActions msg;
  msg.header.header_id = 1;
  msg.header.version = "2.0.0";
  msg.header.manufacturer = kMfg;
  msg.header.serial_number = kSerial;
  vda5050_interfaces::msg::Action a;
  a.action_id = action_id;
  a.action_type = action_type;
  a.blocking_type = "NONE";
  msg.actions.push_back(a);
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

// Stubbed sender — captures call args + canned-result control via
// shared state.
struct SenderStub
{
  std::shared_ptr<std::atomic<int>> calls{
    std::make_shared<std::atomic<int>>(0)};
  std::shared_ptr<vda5050_core::types::InstantActions> last_actions{
    std::make_shared<vda5050_core::types::InstantActions>()};
  std::shared_ptr<std::string> last_mfg{std::make_shared<std::string>()};
  std::shared_ptr<std::string> last_serial{std::make_shared<std::string>()};
  std::shared_ptr<vda5050_core::master::InstantActionAssignmentResult> canned{
    std::make_shared<vda5050_core::master::InstantActionAssignmentResult>()};

  InstantActionsSendService::InstantActionsSender as_callable()
  {
    auto c = calls;
    auto la = last_actions;
    auto lm = last_mfg;
    auto ls = last_serial;
    auto cn = canned;
    return [c, la, lm, ls, cn](
             const std::string& mfg, const std::string& serial,
             const vda5050_core::types::InstantActions& actions)
             -> vda5050_core::master::InstantActionAssignmentResult {
      c->fetch_add(1);
      *lm = mfg;
      *ls = serial;
      *la = actions;
      return *cn;
    };
  }
};

class InstantActionsSendServiceTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("instant_actions_send_service_test_node");
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

  std::shared_ptr<AssignInstantActions::Response> call(
    const std::string& service_name, const std::string& mfg,
    const std::string& serial,
    const vda5050_interfaces::msg::InstantActions& actions,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<AssignInstantActions>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<AssignInstantActions::Request>();
    request->manufacturer = mfg;
    request->serial_number = serial;
    request->instant_actions = actions;
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

TEST_F(InstantActionsSendServiceTest, ServiceNameFollowsConvention)
{
  SenderStub stub;
  InstantActionsSendService svc(node_, stub.as_callable());
  EXPECT_EQ(svc.service_name(), "/vda5050_master/assign_instant_actions");
}

TEST_F(InstantActionsSendServiceTest, CustomNamespaceAppliesToServiceName)
{
  SenderStub stub;
  InstantActionsSendService svc(node_, stub.as_callable(), "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/assign_instant_actions");
}

// =============================================================================
// Decision branches
// =============================================================================

TEST_F(InstantActionsSendServiceTest, ReturnsAssignedOnSuccess)
{
  SenderStub stub;
  stub.canned->decision = vda5050_core::master::InstantActionDecision::ASSIGNED;
  InstantActionsSendService svc(node_, stub.as_callable());

  auto resp = call(
    svc.service_name(), kMfg, kSerial,
    make_actions_msg("a-uuid-1", "stateRequest"));

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->decision, AssignInstantActions::Response::ASSIGNED);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
  EXPECT_TRUE(resp->errors.empty());
  EXPECT_EQ(stub.calls->load(), 1);
}

TEST_F(InstantActionsSendServiceTest, ReturnsAgvNotOnboardedWhenSenderReports)
{
  SenderStub stub;
  stub.canned->decision =
    vda5050_core::master::InstantActionDecision::AGV_NOT_ONBOARDED;
  InstantActionsSendService svc(node_, stub.as_callable());

  auto resp = call(
    svc.service_name(), kMfg, kSerial,
    make_actions_msg("a-uuid-2", "factsheetRequest"));

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->decision, AssignInstantActions::Response::AGV_NOT_ONBOARDED);
}

TEST_F(InstantActionsSendServiceTest, ReturnsInvalidRequestForEmptyMfgOrSerial)
{
  SenderStub stub;
  stub.canned->decision = vda5050_core::master::InstantActionDecision::ASSIGNED;
  InstantActionsSendService svc(node_, stub.as_callable());

  auto r1 = call(
    svc.service_name(), "", kSerial, make_actions_msg("a", "stateRequest"));
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->decision, AssignInstantActions::Response::INVALID_REQUEST);

  auto r2 =
    call(svc.service_name(), kMfg, "", make_actions_msg("a", "stateRequest"));
  ASSERT_NE(r2, nullptr);
  EXPECT_EQ(r2->decision, AssignInstantActions::Response::INVALID_REQUEST);

  EXPECT_EQ(stub.calls->load(), 0);  // sender NOT invoked
}

TEST_F(InstantActionsSendServiceTest, ErrorsAreForwardedOnRejection)
{
  SenderStub stub;
  stub.canned->decision =
    vda5050_core::master::InstantActionDecision::DUPLICATE_ACTION_ID;
  stub.canned->errors = {
    make_error("duplicateActionId", vda5050_core::types::ErrorLevel::WARNING),
    make_error("emptyActionId", vda5050_core::types::ErrorLevel::WARNING)};
  InstantActionsSendService svc(node_, stub.as_callable());

  auto resp = call(
    svc.service_name(), kMfg, kSerial, make_actions_msg("dup", "cancelOrder"));

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(
    resp->decision, AssignInstantActions::Response::DUPLICATE_ACTION_ID);
  ASSERT_EQ(resp->errors.size(), 2u);
  EXPECT_EQ(resp->errors[0].error_type, "duplicateActionId");
  EXPECT_EQ(resp->errors[1].error_type, "emptyActionId");
}

// =============================================================================
// Round-trip: from_msg<>
// =============================================================================

TEST_F(InstantActionsSendServiceTest, ActionsAreConvertedAndForwardedToSender)
{
  SenderStub stub;
  stub.canned->decision = vda5050_core::master::InstantActionDecision::ASSIGNED;
  InstantActionsSendService svc(node_, stub.as_callable());

  auto actions_msg = make_actions_msg("uuid-rt", "factsheetRequest");
  auto resp = call(svc.service_name(), kMfg, kSerial, actions_msg);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->decision, AssignInstantActions::Response::ASSIGNED);
  EXPECT_EQ(*stub.last_mfg, kMfg);
  EXPECT_EQ(*stub.last_serial, kSerial);
  ASSERT_EQ(stub.last_actions->actions.size(), 1u);
  EXPECT_EQ(stub.last_actions->actions.front().action_id, "uuid-rt");
  EXPECT_EQ(stub.last_actions->actions.front().action_type, "factsheetRequest");
}

// =============================================================================
// Defense against future enum drift
// =============================================================================

TEST_F(InstantActionsSendServiceTest, MapsAllInstantActionDecisions)
{
  struct Case
  {
    vda5050_core::master::InstantActionDecision in;
    uint8_t expected;
  };
  const std::vector<Case> cases = {
    {vda5050_core::master::InstantActionDecision::ASSIGNED,
     AssignInstantActions::Response::ASSIGNED},
    {vda5050_core::master::InstantActionDecision::AGV_NOT_ONBOARDED,
     AssignInstantActions::Response::AGV_NOT_ONBOARDED},
    {vda5050_core::master::InstantActionDecision::AGV_OFFLINE,
     AssignInstantActions::Response::AGV_OFFLINE},
    {vda5050_core::master::InstantActionDecision::DUPLICATE_ACTION_ID,
     AssignInstantActions::Response::DUPLICATE_ACTION_ID},
    {vda5050_core::master::InstantActionDecision::AGV_QUEUE_FULL,
     AssignInstantActions::Response::AGV_QUEUE_FULL},
    {vda5050_core::master::InstantActionDecision::HARD_ACTION_BLOCKED,
     AssignInstantActions::Response::HARD_ACTION_BLOCKED},
    {vda5050_core::master::InstantActionDecision::ACTION_BLOCKED_BY_DRIVING,
     AssignInstantActions::Response::ACTION_BLOCKED_BY_DRIVING},
    {vda5050_core::master::InstantActionDecision::AGV_MODE_NOT_AUTO_FOR_ACTION,
     AssignInstantActions::Response::AGV_MODE_NOT_AUTO_FOR_ACTION}};

  for (const auto& c : cases)
  {
    SenderStub stub;
    stub.canned->decision = c.in;
    InstantActionsSendService svc(node_, stub.as_callable());
    auto resp = call(
      svc.service_name(), kMfg, kSerial, make_actions_msg("u", "stateRequest"));
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->decision, c.expected)
      << "Mismatch for vda5050_core::master::InstantActionDecision="
      << static_cast<int>(c.in);
  }
}

}  // namespace test
}  // namespace vda5050_master_ros2
