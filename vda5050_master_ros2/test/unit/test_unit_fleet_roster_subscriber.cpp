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
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_master_ros2/fleet_roster_subscriber.hpp"
#include "vda5050_master_ros2/msg/agv_onboard_spec.hpp"
#include "vda5050_master_ros2/msg/fleet_roster.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

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

// In-memory stand-in for VDA5050Master, exposing only what
// FleetRosterSubscriber needs. Avoids spinning a real master + MQTT
// for what is purely a topic -> batch-API dispatch test.
struct MasterStub
{
  std::mutex mu;
  std::set<std::pair<std::string, std::string>> onboarded;
  std::vector<vda5050_core::master::VDA5050Master::OnboardSpec> last_add;
  std::vector<std::pair<std::string, std::string>> last_remove;

  FleetRosterSubscriber::OnboardedSnapshot snapshot()
  {
    return [this]() {
      std::lock_guard<std::mutex> lock(mu);
      return std::vector<std::pair<std::string, std::string>>(
        onboarded.begin(), onboarded.end());
    };
  }

  FleetRosterSubscriber::OnboardBatcher onboarder()
  {
    return
      [this](
        const std::vector<vda5050_core::master::VDA5050Master::OnboardSpec>&
          specs) -> vda5050_core::master::VDA5050Master::BatchOnboardResult {
        vda5050_core::master::VDA5050Master::BatchOnboardResult r;
        std::lock_guard<std::mutex> lock(mu);
        last_add = specs;
        for (const auto& s : specs)
        {
          auto key = std::make_pair(s.manufacturer, s.serial_number);
          if (onboarded.count(key))
          {
            r.skipped_already_onboarded.push_back(s);
          }
          else
          {
            onboarded.insert(key);
            r.onboarded.push_back(s);
          }
        }
        return r;
      };
  }

  FleetRosterSubscriber::OffboardBatcher offboarder()
  {
    return [this](const std::vector<std::pair<std::string, std::string>>& keys)
             -> std::size_t {
      std::lock_guard<std::mutex> lock(mu);
      last_remove = keys;
      std::size_t removed = 0;
      for (const auto& k : keys)
      {
        removed += onboarded.erase(k);
      }
      return removed;
    };
  }
};

vda5050_master_ros2::msg::AGVOnboardSpec mk_entry(
  const std::string& mfg, const std::string& sn)
{
  vda5050_master_ros2::msg::AGVOnboardSpec e;
  e.manufacturer = mfg;
  e.serial_number = sn;
  e.max_queue_size = 0;
  e.drop_oldest = true;
  return e;
}

class FleetRosterSubscriberTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("fleet_roster_subscriber_test_node");
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

  // Build a latched publisher matching the subscriber's QoS so DM-side
  // publishes are delivered reliably.
  rclcpp::Publisher<vda5050_master_ros2::msg::FleetRoster>::SharedPtr
  make_latched_pub(const std::string& topic)
  {
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    return node_->create_publisher<vda5050_master_ros2::msg::FleetRoster>(
      topic, qos);
  }
};

}  // namespace

TEST_F(FleetRosterSubscriberTest, TopicNameFollowsConvention)
{
  MasterStub stub;
  FleetRosterSubscriber sub(
    node_, stub.snapshot(), stub.onboarder(), stub.offboarder());
  EXPECT_EQ(sub.topic_name(), "/vda5050_master/fleet_roster");
}

TEST_F(FleetRosterSubscriberTest, OnboardsAllFromEmptyMaster)
{
  MasterStub stub;
  FleetRosterSubscriber sub(
    node_, stub.snapshot(), stub.onboarder(), stub.offboarder());
  auto pub = make_latched_pub(sub.topic_name());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  vda5050_master_ros2::msg::FleetRoster msg;
  msg.roster_id = 1;
  msg.publisher_id = "test_dm";
  msg.agvs.push_back(mk_entry("ACME", "AGV01"));
  msg.agvs.push_back(mk_entry("ACME", "AGV02"));
  pub->publish(msg);

  ASSERT_TRUE(wait_for(
    [&] { return sub.last_applied_roster_id() == 1; },
    std::chrono::milliseconds(500)));
  std::lock_guard<std::mutex> lock(stub.mu);
  EXPECT_EQ(stub.onboarded.size(), 2u);
  EXPECT_EQ(stub.last_add.size(), 2u);
  EXPECT_TRUE(stub.last_remove.empty());
  EXPECT_EQ(sub.last_publisher_id(), "test_dm");
}

TEST_F(FleetRosterSubscriberTest, ReconcileAddsAndRemoves)
{
  MasterStub stub;
  stub.onboarded.insert({"ACME", "AGV01"});
  stub.onboarded.insert({"ACME", "AGV02"});

  FleetRosterSubscriber sub(
    node_, stub.snapshot(), stub.onboarder(), stub.offboarder());
  auto pub = make_latched_pub(sub.topic_name());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Desired: keep AGV01, drop AGV02, add BOT01.
  vda5050_master_ros2::msg::FleetRoster msg;
  msg.roster_id = 7;
  msg.agvs.push_back(mk_entry("ACME", "AGV01"));
  msg.agvs.push_back(mk_entry("OTHER", "BOT01"));
  pub->publish(msg);

  ASSERT_TRUE(wait_for(
    [&] { return sub.last_applied_roster_id() == 7; },
    std::chrono::milliseconds(500)));
  std::lock_guard<std::mutex> lock(stub.mu);
  EXPECT_EQ(
    stub.onboarded.count(std::pair<std::string, std::string>{"ACME", "AGV01"}),
    1u);
  EXPECT_EQ(
    stub.onboarded.count(std::pair<std::string, std::string>{"OTHER", "BOT01"}),
    1u);
  EXPECT_EQ(
    stub.onboarded.count(std::pair<std::string, std::string>{"ACME", "AGV02"}),
    0u);
  EXPECT_EQ(stub.last_add.size(), 1u);
  EXPECT_EQ(stub.last_remove.size(), 1u);
}

TEST_F(FleetRosterSubscriberTest, EmptyRosterRemovesAll)
{
  MasterStub stub;
  stub.onboarded.insert({"ACME", "AGV01"});
  stub.onboarded.insert({"ACME", "AGV02"});

  FleetRosterSubscriber sub(
    node_, stub.snapshot(), stub.onboarder(), stub.offboarder());
  auto pub = make_latched_pub(sub.topic_name());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  vda5050_master_ros2::msg::FleetRoster msg;
  msg.roster_id = 2;
  // empty agvs[]
  pub->publish(msg);

  ASSERT_TRUE(wait_for(
    [&] { return sub.last_applied_roster_id() == 2; },
    std::chrono::milliseconds(500)));
  std::lock_guard<std::mutex> lock(stub.mu);
  EXPECT_TRUE(stub.onboarded.empty());
  EXPECT_EQ(stub.last_remove.size(), 2u);
}

TEST_F(FleetRosterSubscriberTest, NoOpRosterMakesNoCalls)
{
  MasterStub stub;
  stub.onboarded.insert({"ACME", "AGV01"});

  FleetRosterSubscriber sub(
    node_, stub.snapshot(), stub.onboarder(), stub.offboarder());
  auto pub = make_latched_pub(sub.topic_name());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  vda5050_master_ros2::msg::FleetRoster msg;
  msg.roster_id = 5;
  msg.agvs.push_back(mk_entry("ACME", "AGV01"));
  pub->publish(msg);

  ASSERT_TRUE(wait_for(
    [&] { return sub.last_applied_roster_id() == 5; },
    std::chrono::milliseconds(500)));
  std::lock_guard<std::mutex> lock(stub.mu);
  EXPECT_EQ(stub.onboarded.size(), 1u);
  EXPECT_TRUE(stub.last_add.empty());
  EXPECT_TRUE(stub.last_remove.empty());
}

TEST_F(FleetRosterSubscriberTest, EmptyEntryFieldsSkipped)
{
  MasterStub stub;
  FleetRosterSubscriber sub(
    node_, stub.snapshot(), stub.onboarder(), stub.offboarder());
  auto pub = make_latched_pub(sub.topic_name());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  vda5050_master_ros2::msg::FleetRoster msg;
  msg.roster_id = 1;
  msg.agvs.push_back(mk_entry("", "AGV01"));  // empty mfg
  msg.agvs.push_back(mk_entry("ACME", ""));   // empty serial
  msg.agvs.push_back(mk_entry("ACME", "AGV02"));
  pub->publish(msg);

  ASSERT_TRUE(wait_for(
    [&] { return sub.last_applied_roster_id() == 1; },
    std::chrono::milliseconds(500)));
  std::lock_guard<std::mutex> lock(stub.mu);
  EXPECT_EQ(stub.onboarded.size(), 1u);
  EXPECT_EQ(stub.last_add.size(), 1u);
}

TEST_F(FleetRosterSubscriberTest, RespectsPerEntryQueueConfig)
{
  MasterStub stub;
  FleetRosterSubscriber sub(
    node_, stub.snapshot(), stub.onboarder(), stub.offboarder());
  auto pub = make_latched_pub(sub.topic_name());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  vda5050_master_ros2::msg::AGVOnboardSpec e1;
  e1.manufacturer = "ACME";
  e1.serial_number = "AGV01";
  e1.max_queue_size = 0;  // master default
  e1.drop_oldest = false;

  vda5050_master_ros2::msg::AGVOnboardSpec e2;
  e2.manufacturer = "ACME";
  e2.serial_number = "AGV02";
  e2.max_queue_size = 50;
  e2.drop_oldest = true;

  vda5050_master_ros2::msg::FleetRoster msg;
  msg.roster_id = 1;
  msg.agvs.push_back(e1);
  msg.agvs.push_back(e2);
  pub->publish(msg);

  ASSERT_TRUE(wait_for(
    [&] { return sub.last_applied_roster_id() == 1; },
    std::chrono::milliseconds(500)));
  std::lock_guard<std::mutex> lock(stub.mu);
  ASSERT_EQ(stub.last_add.size(), 2u);
  // max_queue_size == 0 in the wire means "use master default"; the
  // subscriber leaves the OnboardSpec default (10) in place.
  EXPECT_EQ(stub.last_add[0].max_queue_size, 10u);
  EXPECT_EQ(stub.last_add[0].drop_oldest, false);
  EXPECT_EQ(stub.last_add[1].max_queue_size, 50u);
  EXPECT_EQ(stub.last_add[1].drop_oldest, true);
}

}  // namespace test
}  // namespace vda5050_master_ros2
