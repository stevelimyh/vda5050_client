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

#include "vda5050_master_ros2/get_loaded_map_service.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master_ros2/msg/agv_alignment.hpp"
#include "vda5050_master_ros2/msg/agv_alignment_finding.hpp"

namespace vda5050_master_ros2 {
namespace {

// Convert a system_clock::time_point to a ROS Time. Mirrors the helper
// in device_status_service.cpp; lives here as a private helper so this
// file remains self-contained.
builtin_interfaces::msg::Time to_ros_time(
  const std::chrono::system_clock::time_point& tp)
{
  const auto since_epoch = tp.time_since_epoch();
  const auto secs =
    std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nsecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(secs.count());
  out.nanosec = static_cast<uint32_t>(nsecs.count());
  return out;
}

const char* map_status_to_str(vda5050_core::master::MapStatus s)
{
  switch (s)
  {
    case vda5050_core::master::MapStatus::ENABLED:
      return "ENABLED";
    case vda5050_core::master::MapStatus::DISABLED:
      return "DISABLED";
  }
  return "ENABLED";
}

vda5050_master_ros2::msg::AgvAlignmentFinding finding_to_msg(
  const vda5050_core::master::AlignmentFinding& f)
{
  vda5050_master_ros2::msg::AgvAlignmentFinding out;
  out.severity =
    (f.severity == vda5050_core::master::AlignmentSeverity::ERROR)
      ? vda5050_master_ros2::msg::AgvAlignmentFinding::SEVERITY_ERROR
      : vda5050_master_ros2::msg::AgvAlignmentFinding::SEVERITY_WARNING;
  out.code = f.code;
  out.description = f.description;
  return out;
}

// Split "manufacturer/serial_number" into its two halves. Returns empty
// strings for both on malformed input — defensive against cache keys
// that don't follow the expected pattern.
std::pair<std::string, std::string> split_agv_id(const std::string& id)
{
  const auto slash = id.find('/');
  if (slash == std::string::npos) return {"", ""};
  return {id.substr(0, slash), id.substr(slash + 1)};
}

}  // namespace

std::string GetLoadedMapService::make_service_name(
  const std::string& topic_namespace)
{
  std::string name = "/";
  if (!topic_namespace.empty())
  {
    name += topic_namespace;
    name += "/";
  }
  name += kServiceLeaf;
  return name;
}

GetLoadedMapService::GetLoadedMapService(
  rclcpp::Node::SharedPtr node, MapLookup map_lookup,
  AlignmentLookup alignment_lookup, const std::string& topic_namespace)
: node_(std::move(node)),
  map_lookup_(std::move(map_lookup)),
  alignment_lookup_(std::move(alignment_lookup)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<GetLoadedMap>(
    service_name_, [this](
                     const std::shared_ptr<GetLoadedMap::Request> request,
                     std::shared_ptr<GetLoadedMap::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[GetLoadedMapService] advertised service on {}", service_name_);
}

void GetLoadedMapService::handle_request(
  const std::shared_ptr<GetLoadedMap::Request> /*request*/,
  std::shared_ptr<GetLoadedMap::Response> response)
{
  std::shared_ptr<const vda5050_core::master::Map> snap = map_lookup_();
  if (snap == nullptr)
  {
    response->status = GetLoadedMap::Response::NO_MAP_LOADED;
    return;
  }

  response->status = GetLoadedMap::Response::SUCCESS;
  response->map_id = snap->info.map_id;
  response->map_version = snap->info.map_version;
  response->map_status = map_status_to_str(snap->info.map_status);
  response->map_descriptor = snap->info.map_descriptor;
  response->loaded_at.push_back(to_ros_time(snap->loaded_at));
  response->node_count = static_cast<uint32_t>(snap->nodes.size());
  response->edge_count = static_cast<uint32_t>(snap->edges.size());
  response->source_path = snap->source_path;

  // Per-AGV alignment summary.
  const auto alignment_cache = alignment_lookup_();
  response->agv_alignments.reserve(alignment_cache.size());
  for (const auto& kv : alignment_cache)
  {
    vda5050_master_ros2::msg::AgvAlignment entry;
    auto [mfg, serial] = split_agv_id(kv.first);
    entry.manufacturer = mfg;
    entry.serial_number = serial;
    entry.has_factsheet = true;  // cached entry implies a factsheet was seen
    entry.has_error = kv.second.has_error();
    entry.finding_count = static_cast<uint32_t>(kv.second.findings.size());
    entry.findings.reserve(kv.second.findings.size());
    for (const auto& f : kv.second.findings)
    {
      entry.findings.push_back(finding_to_msg(f));
    }
    response->agv_alignments.push_back(std::move(entry));
  }
}

}  // namespace vda5050_master_ros2
