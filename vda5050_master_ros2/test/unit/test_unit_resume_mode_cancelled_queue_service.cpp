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
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/resume_mode_cancelled_queue_service.hpp"
#include "vda5050_master_ros2/srv/resume_mode_cancelled_queue.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using ResumeModeCancelledQueue =
  vda5050_master_ros2::srv::ResumeModeCancelledQueue;

class ResumeServiceTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("resume_mode_cancelled_queue_test_node");
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

  std::shared_ptr<ResumeModeCancelledQueue::Response> call(
    const std::string& service_name, const std::string& mfg,
    const std::string& serial,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<ResumeModeCancelledQueue>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<ResumeModeCancelledQueue::Request>();
    request->manufacturer = mfg;
    request->serial_number = serial;
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) return nullptr;
    return future.get();
  }
};

}  // namespace

TEST_F(ResumeServiceTest, ServiceNameFollowsConvention)
{
  ResumeModeCancelledQueueService svc(
    node_,
    [](const std::string&, const std::string&)
      -> std::optional<std::pair<std::size_t, std::size_t>> {
      return std::pair<std::size_t, std::size_t>{0u, 0u};
    });
  EXPECT_EQ(svc.service_name(), "/vda5050_master/resume_mode_cancelled_queue");
}

TEST_F(ResumeServiceTest, CustomNamespaceAppliesToServiceName)
{
  ResumeModeCancelledQueueService svc(
    node_,
    [](const std::string&, const std::string&)
      -> std::optional<std::pair<std::size_t, std::size_t>> {
      return std::pair<std::size_t, std::size_t>{0u, 0u};
    },
    "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/resume_mode_cancelled_queue");
}

TEST_F(ResumeServiceTest, ReturnsSuccessWithCounts)
{
  ResumeModeCancelledQueueService svc(
    node_,
    [](const std::string&, const std::string&)
      -> std::optional<std::pair<std::size_t, std::size_t>> {
      return std::pair<std::size_t, std::size_t>{2u, 1u};
    });
  auto resp = call(svc.service_name(), kMfg, kSerial);
  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, ResumeModeCancelledQueue::Response::SUCCESS);
  EXPECT_EQ(resp->orders_resumed, 2u);
  EXPECT_EQ(resp->actions_resumed, 1u);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
}

TEST_F(ResumeServiceTest, ReturnsAgvNotOnboardedForMissingAgv)
{
  ResumeModeCancelledQueueService svc(
    node_,
    [](const std::string&, const std::string&)
      -> std::optional<std::pair<std::size_t, std::size_t>> {
      return std::nullopt;  // vda5050_core::master::AGV not onboarded
    });
  auto resp = call(svc.service_name(), kMfg, kSerial);
  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(
    resp->status, ResumeModeCancelledQueue::Response::AGV_NOT_ONBOARDED);
}

TEST_F(ResumeServiceTest, ReturnsInvalidRequestForEmptyMfgOrSerial)
{
  std::atomic<int> handler_calls{0};
  ResumeModeCancelledQueueService svc(
    node_,
    [&handler_calls](const std::string&, const std::string&)
      -> std::optional<std::pair<std::size_t, std::size_t>> {
      handler_calls.fetch_add(1);
      return std::pair<std::size_t, std::size_t>{0u, 0u};
    });

  auto r1 = call(svc.service_name(), "", kSerial);
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->status, ResumeModeCancelledQueue::Response::INVALID_REQUEST);

  auto r2 = call(svc.service_name(), kMfg, "");
  ASSERT_NE(r2, nullptr);
  EXPECT_EQ(r2->status, ResumeModeCancelledQueue::Response::INVALID_REQUEST);

  EXPECT_EQ(handler_calls.load(), 0);
}

}  // namespace test
}  // namespace vda5050_master_ros2
