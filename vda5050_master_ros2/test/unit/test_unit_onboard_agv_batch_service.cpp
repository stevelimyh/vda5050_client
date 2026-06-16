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
#include "vda5050_master_ros2/onboard_agv_batch_service.hpp"
#include "vda5050_master_ros2/srv/onboard_agv_batch.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

using OnboardAGVBatch = vda5050_master_ros2::srv::OnboardAGVBatch;
using vda5050_core::master::VDA5050Master;

class OnboardAGVBatchServiceTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("onboard_agv_batch_test_node");
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
};

vda5050_master_ros2::msg::AGVOnboardSpec mk_entry(
  const std::string& mfg, const std::string& sn)
{
  vda5050_master_ros2::msg::AGVOnboardSpec e;
  e.manufacturer = mfg;
  e.serial_number = sn;
  return e;
}

}  // namespace

TEST_F(OnboardAGVBatchServiceTest, ServiceNameFollowsConvention)
{
  auto batcher = [](const std::vector<VDA5050Master::OnboardSpec>&) {
    return VDA5050Master::BatchOnboardResult{};
  };
  OnboardAGVBatchService svc(node_, batcher);
  EXPECT_EQ(svc.service_name(), "/vda5050_master/onboard_agv_batch");
}

TEST_F(OnboardAGVBatchServiceTest, ProcessesEachEntryThroughBatcher)
{
  auto batcher = [](const std::vector<VDA5050Master::OnboardSpec>& specs) {
    VDA5050Master::BatchOnboardResult r;
    for (std::size_t i = 0; i < specs.size(); ++i)
    {
      // First two onboard, third fails (mimics partial success).
      if (i < 2)
        r.onboarded.push_back(specs[i]);
      else
        r.failed.push_back(specs[i]);
    }
    return r;
  };
  OnboardAGVBatchService svc(node_, batcher);

  auto client = node_->create_client<OnboardAGVBatch>(svc.service_name());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<OnboardAGVBatch::Request>();
  req->agvs = {
    mk_entry("ACME", "A"), mk_entry("ACME", "B"), mk_entry("ACME", "C")};
  auto fut = client->async_send_request(req);
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto resp = fut.get();
  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->onboarded.size(), 2u);
  EXPECT_EQ(resp->failed.size(), 1u);
  EXPECT_EQ(resp->failed[0].serial_number, "C");
}

TEST_F(OnboardAGVBatchServiceTest, EmptyRequestIsHandledCleanly)
{
  std::atomic<int> calls{0};
  auto batcher = [&calls](const std::vector<VDA5050Master::OnboardSpec>& s) {
    calls.fetch_add(1);
    EXPECT_TRUE(s.empty());
    return VDA5050Master::BatchOnboardResult{};
  };
  OnboardAGVBatchService svc(node_, batcher);

  auto client = node_->create_client<OnboardAGVBatch>(svc.service_name());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<OnboardAGVBatch::Request>();
  auto fut = client->async_send_request(req);
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto resp = fut.get();
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->onboarded.empty());
  EXPECT_TRUE(resp->failed.empty());
  EXPECT_EQ(calls.load(), 1);
}

}  // namespace test
}  // namespace vda5050_master_ros2
