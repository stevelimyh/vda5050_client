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
#include <unordered_map>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/map/map.hpp"
#include "vda5050_core/master/validation/factsheet_alignment.hpp"
#include "vda5050_master_ros2/get_loaded_map_service.hpp"
#include "vda5050_master_ros2/srv/get_loaded_map.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

using GetLoadedMap = vda5050_master_ros2::srv::GetLoadedMap;

// Build a small map with two nodes + one edge.
std::shared_ptr<const vda5050_core::master::Map> make_test_map()
{
  vda5050_core::master::Map m;
  m.info.map_id = "warehouse_floor1";
  m.info.map_version = "1.0";
  m.info.map_status = vda5050_core::master::MapStatus::ENABLED;
  m.info.map_descriptor = "Floor 1 demo";
  m.source_path = "/etc/vda5050/floor1.json";
  vda5050_core::master::MapNode n0;
  n0.node_id = "N0";
  vda5050_core::master::MapNode n1;
  n1.node_id = "N1";
  m.nodes = {n0, n1};
  vda5050_core::master::MapEdge e;
  e.edge_id = "E1";
  e.start_node_id = "N0";
  e.end_node_id = "N1";
  m.edges = {e};
  m.loaded_at = std::chrono::system_clock::now();
  return std::make_shared<const vda5050_core::master::Map>(std::move(m));
}

// Build an alignment cache with one entry: vda5050_core::master::AGV "ACME/AGV01" with one
// ERROR finding for SpeedExceedsCapability.
std::unordered_map<std::string, vda5050_core::master::FactsheetAlignmentResult>
make_test_alignment_cache()
{
  std::unordered_map<
    std::string, vda5050_core::master::FactsheetAlignmentResult>
    out;
  vda5050_core::master::FactsheetAlignmentResult r;
  r.findings.push_back(
    {vda5050_core::master::AlignmentSeverity::ERROR, "SpeedExceedsCapability",
     "edge E1 max_speed=2.0 > factsheet speedMax=1.5"});
  out.emplace("ACME/AGV01", std::move(r));
  return out;
}

class GetLoadedMapServiceTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("get_loaded_map_service_test_node");
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

  std::shared_ptr<GetLoadedMap::Response> call(
    const std::string& service_name,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    auto client = node_->create_client<GetLoadedMap>(service_name);
    if (!client->wait_for_service(timeout)) return nullptr;
    auto request = std::make_shared<GetLoadedMap::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) return nullptr;
    return future.get();
  }
};

}  // namespace

TEST_F(GetLoadedMapServiceTest, ServiceNameFollowsConvention)
{
  GetLoadedMapService svc(
    node_,
    []() -> std::shared_ptr<const vda5050_core::master::Map> {
      return nullptr;
    },
    []() {
      return std::unordered_map<
        std::string, vda5050_core::master::FactsheetAlignmentResult>{};
    });
  EXPECT_EQ(svc.service_name(), "/vda5050_master/get_loaded_map");
}

TEST_F(GetLoadedMapServiceTest, CustomNamespaceAppliesToServiceName)
{
  GetLoadedMapService svc(
    node_,
    []() -> std::shared_ptr<const vda5050_core::master::Map> {
      return nullptr;
    },
    []() {
      return std::unordered_map<
        std::string, vda5050_core::master::FactsheetAlignmentResult>{};
    },
    "my_master");
  EXPECT_EQ(svc.service_name(), "/my_master/get_loaded_map");
}

TEST_F(GetLoadedMapServiceTest, ReturnsNoMapLoadedWhenAbsent)
{
  GetLoadedMapService svc(
    node_,
    []() -> std::shared_ptr<const vda5050_core::master::Map> {
      return nullptr;
    },
    []() {
      return std::unordered_map<
        std::string, vda5050_core::master::FactsheetAlignmentResult>{};
    });

  auto resp = call(svc.service_name());
  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, GetLoadedMap::Response::NO_MAP_LOADED);
  EXPECT_TRUE(resp->map_id.empty());
  EXPECT_EQ(resp->node_count, 0u);
  EXPECT_EQ(resp->edge_count, 0u);
  EXPECT_TRUE(resp->agv_alignments.empty());
}

TEST_F(GetLoadedMapServiceTest, ReturnsLoadedMapWithNodeEdgeCounts)
{
  auto m = make_test_map();
  GetLoadedMapService svc(
    node_, [m]() { return m; },
    []() {
      return std::unordered_map<
        std::string, vda5050_core::master::FactsheetAlignmentResult>{};
    });

  auto resp = call(svc.service_name());
  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, GetLoadedMap::Response::SUCCESS);
  EXPECT_EQ(resp->map_id, "warehouse_floor1");
  EXPECT_EQ(resp->map_version, "1.0");
  EXPECT_EQ(resp->map_status, "ENABLED");
  EXPECT_EQ(resp->map_descriptor, "Floor 1 demo");
  EXPECT_EQ(resp->source_path, "/etc/vda5050/floor1.json");
  EXPECT_EQ(resp->node_count, 2u);
  EXPECT_EQ(resp->edge_count, 1u);
}

TEST_F(GetLoadedMapServiceTest, IncludesAlignmentReportPerOnboardedAgv)
{
  auto m = make_test_map();
  auto cache = make_test_alignment_cache();
  GetLoadedMapService svc(
    node_, [m]() { return m; }, [cache]() { return cache; });

  auto resp = call(svc.service_name());
  ASSERT_NE(resp, nullptr) << "service call timed out";
  EXPECT_EQ(resp->status, GetLoadedMap::Response::SUCCESS);
  ASSERT_EQ(resp->agv_alignments.size(), 1u);

  const auto& a = resp->agv_alignments[0];
  EXPECT_EQ(a.manufacturer, "ACME");
  EXPECT_EQ(a.serial_number, "AGV01");
  EXPECT_TRUE(a.has_factsheet);
  EXPECT_TRUE(a.has_error);
  EXPECT_EQ(a.finding_count, 1u);
  ASSERT_EQ(a.findings.size(), 1u);
  EXPECT_EQ(a.findings[0].code, "SpeedExceedsCapability");
  EXPECT_EQ(
    a.findings[0].severity,
    vda5050_master_ros2::msg::AgvAlignmentFinding::SEVERITY_ERROR);
}

}  // namespace test
}  // namespace vda5050_master_ros2
