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
#include "vda5050_master_ros2/offboard_agv_service.hpp"
#include "vda5050_master_ros2/srv/offboard_agv.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using OffboardAGV = vda5050_master_ros2::srv::OffboardAGV;

struct BatcherStub
{
  std::shared_ptr<std::atomic<int>> calls{
    std::make_shared<std::atomic<int>>(0)};
  std::shared_ptr<std::string> last_mfg{std::make_shared<std::string>()};
  std::shared_ptr<std::string> last_serial{std::make_shared<std::string>()};
  // Number of AGVs the batcher should report as removed.
  std::shared_ptr<std::atomic<std::size_t>> canned_removed{
    std::make_shared<std::atomic<std::size_t>>(1)};

  OffboardAGVService::OffboardBatcher as_callable()
  {
    auto c = calls;
    auto lm = last_mfg;
    auto ls = last_serial;
    auto cn = canned_removed;
    return [c, lm, ls,
            cn](const std::vector<std::pair<std::string, std::string>>& keys)
             -> std::size_t {
      c->fetch_add(1);
      if (!keys.empty())
      {
        *lm = keys[0].first;
        *ls = keys[0].second;
      }
      return cn->load();
    };
  }
};

class OffboardAGVServiceTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("offboard_agv_service_test_node");
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

  std::shared_ptr<OffboardAGV::Response> call(
    const std::string& service_name, const std::string& mfg,
    const std::string& serial,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<OffboardAGV>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<OffboardAGV::Request>();
    request->agv.manufacturer = mfg;
    request->agv.serial_number = serial;
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }
};

}  // namespace

TEST_F(OffboardAGVServiceTest, ServiceNameFollowsConvention)
{
  BatcherStub stub;
  OffboardAGVService svc(node_, stub.as_callable());
  EXPECT_EQ(svc.service_name(), "/vda5050_master/offboard_agv");
}

TEST_F(OffboardAGVServiceTest, CustomNamespaceAppliesToServiceName)
{
  BatcherStub stub;
  OffboardAGVService svc(node_, stub.as_callable(), "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/offboard_agv");
}

TEST_F(OffboardAGVServiceTest, ReturnsSuccessForOnboardedAGV)
{
  BatcherStub stub;
  stub.canned_removed->store(1);
  OffboardAGVService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, OffboardAGV::Response::SUCCESS);
  EXPECT_EQ(resp->manufacturer, kMfg);
  EXPECT_EQ(resp->serial_number, kSerial);
  EXPECT_EQ(stub.calls->load(), 1);
}

TEST_F(OffboardAGVServiceTest, ReturnsNotOnboardedForUnknownAGV)
{
  BatcherStub stub;
  stub.canned_removed->store(0);
  OffboardAGVService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->status, OffboardAGV::Response::NOT_ONBOARDED);
}

TEST_F(OffboardAGVServiceTest, ReturnsInvalidRequestForEmptyMfgOrSerial)
{
  BatcherStub stub;
  OffboardAGVService svc(node_, stub.as_callable());

  auto r1 = call(svc.service_name(), "", kSerial);
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->status, OffboardAGV::Response::INVALID_REQUEST);

  auto r2 = call(svc.service_name(), kMfg, "");
  ASSERT_NE(r2, nullptr);
  EXPECT_EQ(r2->status, OffboardAGV::Response::INVALID_REQUEST);

  EXPECT_EQ(stub.calls->load(), 0);
}

TEST_F(OffboardAGVServiceTest, ForwardsArgsToBatcher)
{
  BatcherStub stub;
  stub.canned_removed->store(1);
  OffboardAGVService svc(node_, stub.as_callable());

  auto resp = call(svc.service_name(), kMfg, kSerial);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(*stub.last_mfg, kMfg);
  EXPECT_EQ(*stub.last_serial, kSerial);
}

}  // namespace test
}  // namespace vda5050_master_ros2
