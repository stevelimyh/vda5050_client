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
#include <cstddef>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/offboard_agv_batch_service.hpp"
#include "vda5050_master_ros2/srv/offboard_agv_batch.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

using OffboardAGVBatch = vda5050_master_ros2::srv::OffboardAGVBatch;

class OffboardAGVBatchServiceTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("offboard_agv_batch_test_node");
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

}  // namespace

TEST_F(OffboardAGVBatchServiceTest, ServiceNameFollowsConvention)
{
  auto batcher =
    [](const std::vector<std::pair<std::string, std::string>>&) -> std::size_t {
    return 0;
  };
  OffboardAGVBatchService svc(node_, batcher);
  EXPECT_EQ(svc.service_name(), "/vda5050_master/offboard_agv_batch");
}

TEST_F(OffboardAGVBatchServiceTest, ForwardsKeysAndReportsCount)
{
  std::atomic<int> calls{0};
  auto batcher =
    [&calls](const std::vector<std::pair<std::string, std::string>>& keys)
    -> std::size_t {
    calls.fetch_add(1);
    // Mimic master.offboard_agv_batch — 2 of 3 keys were onboarded.
    return std::min<std::size_t>(keys.size(), 2);
  };
  OffboardAGVBatchService svc(node_, batcher);

  auto client = node_->create_client<OffboardAGVBatch>(svc.service_name());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<OffboardAGVBatch::Request>();
  for (const auto& s : {"A", "B", "C"})
  {
    vda5050_master_ros2::msg::AGVKey k;
    k.manufacturer = "ACME";
    k.serial_number = s;
    req->agvs.push_back(k);
  }
  auto fut = client->async_send_request(req);
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);

  auto resp = fut.get();
  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->offboarded_count, 2u);
  EXPECT_EQ(calls.load(), 1);
}

}  // namespace test
}  // namespace vda5050_master_ros2
