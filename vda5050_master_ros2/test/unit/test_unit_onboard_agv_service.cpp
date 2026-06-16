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
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/onboard_agv_service.hpp"
#include "vda5050_master_ros2/srv/onboard_agv.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using OnboardAGV = vda5050_master_ros2::srv::OnboardAGV;
using vda5050_core::master::VDA5050Master;

// Where the canned batch result lands. Encodes which of the three
// buckets (onboarded / skipped / failed) the only entry should fall
// into, so each test can dial in the outcome it wants to verify.
enum class CannedOutcome
{
  kOnboarded,
  kAlreadyOnboarded,
  kFailed
};

// Stubbed batcher — captures call args + canned-result control via
// shared state.
struct BatcherStub
{
  std::shared_ptr<std::atomic<int>> calls{
    std::make_shared<std::atomic<int>>(0)};
  std::shared_ptr<std::string> last_mfg{std::make_shared<std::string>()};
  std::shared_ptr<std::string> last_serial{std::make_shared<std::string>()};
  std::shared_ptr<std::atomic<std::size_t>> last_qs{
    std::make_shared<std::atomic<std::size_t>>(0)};
  std::shared_ptr<std::atomic<bool>> last_drop{
    std::make_shared<std::atomic<bool>>(false)};
  std::shared_ptr<CannedOutcome> canned{
    std::make_shared<CannedOutcome>(CannedOutcome::kOnboarded)};

  OnboardAGVService::OnboardBatcher as_callable()
  {
    auto c = calls;
    auto lm = last_mfg;
    auto ls = last_serial;
    auto lq = last_qs;
    auto ld = last_drop;
    auto cn = canned;
    return [c, lm, ls, lq, ld,
            cn](const std::vector<VDA5050Master::OnboardSpec>& specs)
             -> VDA5050Master::BatchOnboardResult {
      c->fetch_add(1);
      VDA5050Master::BatchOnboardResult r;
      if (specs.empty()) return r;
      const auto& s = specs[0];
      *lm = s.manufacturer;
      *ls = s.serial_number;
      lq->store(s.max_queue_size);
      ld->store(s.drop_oldest);
      switch (*cn)
      {
        case CannedOutcome::kOnboarded:
          r.onboarded.push_back(s);
          break;
        case CannedOutcome::kAlreadyOnboarded:
          r.skipped_already_onboarded.push_back(s);
          break;
        case CannedOutcome::kFailed:
          r.failed.push_back(s);
          break;
      }
      return r;
    };
  }
};

class OnboardAGVServiceTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("onboard_agv_service_test_node");
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

  std::shared_ptr<OnboardAGV::Response> call(
    const std::string& service_name, const std::string& mfg,
    const std::string& serial, uint32_t max_queue_size = 0,
    bool drop_oldest = true,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<OnboardAGV>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<OnboardAGV::Request>();
    request->agv.manufacturer = mfg;
    request->agv.serial_number = serial;
    request->agv.max_queue_size = max_queue_size;
    request->agv.drop_oldest = drop_oldest;
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }
};

}  // namespace

TEST_F(OnboardAGVServiceTest, ServiceNameFollowsConvention)
{
  BatcherStub stub;
  OnboardAGVService svc(node_, stub.as_callable());
  EXPECT_EQ(svc.service_name(), "/vda5050_master/onboard_agv");
}

TEST_F(OnboardAGVServiceTest, CustomNamespaceAppliesToServiceName)
{
  BatcherStub stub;
  OnboardAGVService svc(node_, stub.as_callable(), "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/onboard_agv");
}

TEST_F(OnboardAGVServiceTest, ReturnsSuccessForFreshOnboard)
{
  BatcherStub stub;
  *stub.canned = CannedOutcome::kOnboarded;
  OnboardAGVService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, OnboardAGV::Response::SUCCESS);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
  EXPECT_EQ(stub.calls->load(), 1);
}

TEST_F(OnboardAGVServiceTest, ReturnsAlreadyOnboardedForDuplicate)
{
  BatcherStub stub;
  *stub.canned = CannedOutcome::kAlreadyOnboarded;
  OnboardAGVService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, OnboardAGV::Response::ALREADY_ONBOARDED);
}

TEST_F(OnboardAGVServiceTest, ReturnsInvalidRequestForEmptyMfgOrSerial)
{
  BatcherStub stub;
  *stub.canned = CannedOutcome::kFailed;
  OnboardAGVService svc(node_, stub.as_callable());

  // The batcher itself classifies empty-mfg / empty-serial as failed.
  auto r1 = call(svc.service_name(), "", kSerial);
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->status, OnboardAGV::Response::INVALID_REQUEST);

  auto r2 = call(svc.service_name(), kMfg, "");
  ASSERT_NE(r2, nullptr);
  EXPECT_EQ(r2->status, OnboardAGV::Response::INVALID_REQUEST);
}

TEST_F(OnboardAGVServiceTest, ForwardsArgsToBatcherWithDefaultQueueSize)
{
  BatcherStub stub;
  *stub.canned = CannedOutcome::kOnboarded;
  OnboardAGVService svc(node_, stub.as_callable());

  // Request max_queue_size=0 sentinel — service should leave the
  // OnboardSpec at the master-side default (10).
  auto resp = call(
    svc.service_name(), kMfg, kSerial, /*max_queue_size=*/0,
    /*drop_oldest=*/false);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, OnboardAGV::Response::SUCCESS);
  EXPECT_EQ(*stub.last_mfg, kMfg);
  EXPECT_EQ(*stub.last_serial, kSerial);
  EXPECT_EQ(stub.last_qs->load(), 10u);  // OnboardSpec default
  EXPECT_EQ(stub.last_drop->load(), false);
}

TEST_F(OnboardAGVServiceTest, ForwardsExplicitQueueSize)
{
  BatcherStub stub;
  *stub.canned = CannedOutcome::kOnboarded;
  OnboardAGVService svc(node_, stub.as_callable());

  auto resp = call(
    svc.service_name(), kMfg, kSerial, /*max_queue_size=*/42,
    /*drop_oldest=*/true);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(stub.last_qs->load(), 42u);
  EXPECT_EQ(stub.last_drop->load(), true);
}

}  // namespace test
}  // namespace vda5050_master_ros2
