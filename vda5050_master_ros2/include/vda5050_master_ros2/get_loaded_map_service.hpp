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

#ifndef VDA5050_MASTER_ROS2__GET_LOADED_MAP_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__GET_LOADED_MAP_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/map/map.hpp"
#include "vda5050_core/master/validation/factsheet_alignment.hpp"
#include "vda5050_master_ros2/srv/get_loaded_map.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// GetLoadedMapService (Task #39 — operability surface).
// =============================================================================
//
// Synchronous ROS 2 service returning the master's currently-loaded
// topology map metadata + per-AGV alignment status. Operator
// diagnostics — answers "what map is the master running with, and is
// every onboarded AGV compatible with it?" without scraping logs.
//
// Service name: /<topic_namespace>/get_loaded_map
//   Default <topic_namespace> = "vda5050_master".
//
// **Decoupled from VDA5050Master via two callables**:
//   - MapLookup     — returns the active shared_ptr<const Map> (or
//                     nullptr when no map is loaded).
//   - AlignmentLookup — returns a snapshot of the per-AGV alignment cache.
// Same pattern as DeviceStatusService — keeps unit tests simple (a small
// lambda is enough; no Master construction required).
//
// **Response shape** (vda5050_master/srv/GetLoadedMap):
//   status == NO_MAP_LOADED  → all string / count / time fields zeroed,
//                              agv_alignments empty.
//   status == SUCCESS        → metadata populated, agv_alignments has
//                              one AgvAlignment per cached entry.

class GetLoadedMapService
{
public:
  /// Service name leaf.
  static constexpr const char* kServiceLeaf = "get_loaded_map";

  /// Returns the master's active map snapshot, or nullptr if none loaded.
  using MapLookup =
    std::function<std::shared_ptr<const vda5050_core::master::Map>()>;

  /// Returns a snapshot of the alignment cache (agv_id → result).
  using AlignmentLookup = std::function<std::unordered_map<
    std::string, vda5050_core::master::FactsheetAlignmentResult>()>;

  GetLoadedMapService(
    rclcpp::Node::SharedPtr node, MapLookup map_lookup,
    AlignmentLookup alignment_lookup,
    const std::string& topic_namespace = "vda5050_master");

  ~GetLoadedMapService() = default;
  GetLoadedMapService(const GetLoadedMapService&) = delete;
  GetLoadedMapService& operator=(const GetLoadedMapService&) = delete;
  GetLoadedMapService(GetLoadedMapService&&) = delete;
  GetLoadedMapService& operator=(GetLoadedMapService&&) = delete;

  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using GetLoadedMap = vda5050_master_ros2::srv::GetLoadedMap;

  void handle_request(
    const std::shared_ptr<GetLoadedMap::Request> request,
    std::shared_ptr<GetLoadedMap::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  MapLookup map_lookup_;
  AlignmentLookup alignment_lookup_;
  std::string service_name_;
  rclcpp::Service<GetLoadedMap>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__GET_LOADED_MAP_SERVICE_HPP_
