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
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/master_connection_publisher.hpp"
#include "vda5050_master_ros2/msg/master_connection.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

using MasterConnection = vda5050_master_ros2::msg::MasterConnection;

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

class Collector
{
public:
  void attach(rclcpp::Node::SharedPtr node, const std::string& topic)
  {
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    sub_ = node->create_subscription<MasterConnection>(
      topic, qos, [this](MasterConnection::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mu_);
        received_.push_back(*msg);
      });
  }

  std::size_t count()
  {
    std::lock_guard<std::mutex> lock(mu_);
    return received_.size();
  }

  MasterConnection latest()
  {
    std::lock_guard<std::mutex> lock(mu_);
    return received_.back();
  }

private:
  std::mutex mu_;
  std::vector<MasterConnection> received_;
  rclcpp::Subscription<MasterConnection>::SharedPtr sub_;
};

class MasterConnectionPublisherTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("master_connection_test_node");
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

TEST_F(MasterConnectionPublisherTest, TopicNameAndIdentityEcho)
{
  MasterConnectionPublisher pub(
    node_, "master-001", "vda5050_master_ros2-2.0.0", [] { return true; },
    [] { return 0u; });
  EXPECT_EQ(pub.topic_name(), "/vda5050_master/master_connection");

  Collector collector;
  collector.attach(node_, pub.topic_name());
  ASSERT_TRUE(
    wait_for([&] { return collector.count() >= 1; }, std::chrono::seconds(1)));
  auto m = collector.latest();
  EXPECT_EQ(m.master_id, "master-001");
  EXPECT_EQ(m.master_version, "vda5050_master_ros2-2.0.0");
  EXPECT_EQ(m.state, MasterConnection::STARTING);
  EXPECT_TRUE(m.broker_connected);
}

TEST_F(MasterConnectionPublisherTest, InitialPublishIsStarting)
{
  MasterConnectionPublisher pub(
    node_, "m", "v", [] { return false; }, [] { return 0u; });
  EXPECT_EQ(pub.current_state(), MasterConnection::STARTING);

  Collector collector;
  collector.attach(node_, pub.topic_name());
  ASSERT_TRUE(
    wait_for([&] { return collector.count() >= 1; }, std::chrono::seconds(1)));
  EXPECT_EQ(collector.latest().state, MasterConnection::STARTING);
}

TEST_F(MasterConnectionPublisherTest, SetStatePublishesOnChange)
{
  MasterConnectionPublisher pub(
    node_, "m", "v", [] { return true; }, [] { return 0u; });

  Collector collector;
  collector.attach(node_, pub.topic_name());
  ASSERT_TRUE(
    wait_for([&] { return collector.count() >= 1; }, std::chrono::seconds(1)));

  pub.set_state(MasterConnection::READY);
  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 2; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(collector.latest().state, MasterConnection::READY);
  EXPECT_EQ(pub.current_state(), MasterConnection::READY);
}

TEST_F(MasterConnectionPublisherTest, SetStateNoopOnUnchanged)
{
  MasterConnectionPublisher pub(
    node_, "m", "v", [] { return true; }, [] { return 0u; });

  Collector collector;
  collector.attach(node_, pub.topic_name());
  ASSERT_TRUE(
    wait_for([&] { return collector.count() >= 1; }, std::chrono::seconds(1)));

  pub.set_state(MasterConnection::STARTING);  // unchanged
  // give the publisher time to misbehave; expect still 1 message
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(collector.count(), 1u);
}

TEST_F(MasterConnectionPublisherTest, LookupsEchoIntoPayload)
{
  std::atomic<bool> broker_up{false};
  std::atomic<std::uint32_t> count{0};
  MasterConnectionPublisher pub(
    node_, "m", "v", [&] { return broker_up.load(); },
    [&] { return count.load(); });

  Collector collector;
  collector.attach(node_, pub.topic_name());
  ASSERT_TRUE(
    wait_for([&] { return collector.count() >= 1; }, std::chrono::seconds(1)));
  EXPECT_FALSE(collector.latest().broker_connected);
  EXPECT_EQ(collector.latest().onboarded_agv_count, 0u);

  broker_up = true;
  count = 5;
  pub.set_state(MasterConnection::READY);
  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 2; }, std::chrono::milliseconds(500)));
  EXPECT_TRUE(collector.latest().broker_connected);
  EXPECT_EQ(collector.latest().onboarded_agv_count, 5u);
}

TEST_F(MasterConnectionPublisherTest, LatchedDeliveryToLateSubscriber)
{
  MasterConnectionPublisher pub(
    node_, "m", "v", [] { return true; }, [] { return 0u; });
  pub.set_state(MasterConnection::READY);
  // Allow the READY message to be queued before we attach.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  Collector collector;
  collector.attach(node_, pub.topic_name());
  ASSERT_TRUE(
    wait_for([&] { return collector.count() >= 1; }, std::chrono::seconds(1)));
  EXPECT_EQ(collector.latest().state, MasterConnection::READY);
}

}  // namespace test
}  // namespace vda5050_master_ros2
