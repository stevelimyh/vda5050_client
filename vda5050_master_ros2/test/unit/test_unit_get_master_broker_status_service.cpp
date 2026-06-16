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

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/get_master_broker_status_service.hpp"
#include "vda5050_master_ros2/srv/get_master_broker_status.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

using GetMasterBrokerStatus = vda5050_master_ros2::srv::GetMasterBrokerStatus;

class GetMasterBrokerStatusServiceTest : public ::testing::Test
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
    node_ =
      rclcpp::Node::make_shared("get_master_broker_status_service_test_node");
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

  std::shared_ptr<GetMasterBrokerStatus::Response> call(
    const std::string& service_name,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<GetMasterBrokerStatus>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<GetMasterBrokerStatus::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) return nullptr;
    return future.get();
  }
};

}  // namespace

TEST_F(GetMasterBrokerStatusServiceTest, ServiceNameFollowsConvention)
{
  GetMasterBrokerStatusService svc(
    node_, []() { return GetMasterBrokerStatusService::StatusSnapshot{}; });
  EXPECT_EQ(svc.service_name(), "/vda5050_master/get_master_broker_status");
}

TEST_F(GetMasterBrokerStatusServiceTest, CustomNamespaceAppliesToServiceName)
{
  GetMasterBrokerStatusService svc(
    node_, []() { return GetMasterBrokerStatusService::StatusSnapshot{}; },
    "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/get_master_broker_status");
}

TEST_F(GetMasterBrokerStatusServiceTest, NeverConnectedReportsZeroes)
{
  // Snapshot before any successful connect: connected=false,
  // last_disconnect_at=nullopt, reconnect_count=0.
  GetMasterBrokerStatusService svc(
    node_, []() { return GetMasterBrokerStatusService::StatusSnapshot{}; });

  auto resp = call(svc.service_name());
  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_FALSE(resp->connected);
  EXPECT_EQ(resp->reconnect_count, 0u);
  EXPECT_TRUE(resp->last_disconnect_at.empty());
}

TEST_F(GetMasterBrokerStatusServiceTest, ConnectedSnapshotPropagates)
{
  GetMasterBrokerStatusService::StatusSnapshot snap;
  snap.connected = true;
  snap.reconnect_count = 7;
  // last_disconnect_at is left empty — initial connect after process
  // start typically has no prior disconnect.
  GetMasterBrokerStatusService svc(node_, [snap]() { return snap; });

  auto resp = call(svc.service_name());
  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_TRUE(resp->connected);
  EXPECT_EQ(resp->reconnect_count, 7u);
  EXPECT_TRUE(resp->last_disconnect_at.empty());
}

TEST_F(GetMasterBrokerStatusServiceTest, DisconnectedSnapshotIncludesTimestamp)
{
  GetMasterBrokerStatusService::StatusSnapshot snap;
  snap.connected = false;
  snap.reconnect_count = 3;
  // Concrete instant we can verify byte-for-byte against the wire
  // format. system_clock::time_point(seconds(1700000000)) → epoch +
  // 1700000000s, zero nanos.
  snap.last_disconnect_at =
    std::chrono::system_clock::time_point{std::chrono::seconds{1700000000}};
  GetMasterBrokerStatusService svc(node_, [snap]() { return snap; });

  auto resp = call(svc.service_name());
  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_FALSE(resp->connected);
  EXPECT_EQ(resp->reconnect_count, 3u);
  ASSERT_EQ(resp->last_disconnect_at.size(), 1u);
  EXPECT_EQ(resp->last_disconnect_at[0].sec, 1700000000);
  EXPECT_EQ(resp->last_disconnect_at[0].nanosec, 0u);
}

}  // namespace test
}  // namespace vda5050_master_ros2
