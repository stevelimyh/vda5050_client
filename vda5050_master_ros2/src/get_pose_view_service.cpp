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

#include "vda5050_master_ros2/get_pose_view_service.hpp"

#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/pose_view.hpp"
#include "vda5050_master_ros2/pose_view_builder.hpp"

namespace vda5050_master_ros2 {

std::string GetPoseViewService::make_service_name(
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

GetPoseViewService::GetPoseViewService(
  rclcpp::Node::SharedPtr node, AgvLookup agv_lookup,
  const std::string& topic_namespace)
: node_(std::move(node)),
  agv_lookup_(std::move(agv_lookup)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<GetPoseView>(
    service_name_, [this](
                     const std::shared_ptr<GetPoseView::Request> request,
                     std::shared_ptr<GetPoseView::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[GetPoseViewService] advertised service on {}", service_name_);
}

void GetPoseViewService::handle_request(
  const std::shared_ptr<GetPoseView::Request> request,
  std::shared_ptr<GetPoseView::Response> response)
{
  // Always echo the request key for async-batching correlation.
  response->manufacturer = request->manufacturer;
  response->serial_number = request->serial_number;

  if (request->manufacturer.empty() || request->serial_number.empty())
  {
    response->status = GetPoseView::Response::INVALID_REQUEST;
    return;
  }

  std::shared_ptr<vda5050_core::master::AGV> agv =
    agv_lookup_(request->manufacturer, request->serial_number);
  if (!agv)
  {
    response->status = GetPoseView::Response::AGV_NOT_ONBOARDED;
    return;
  }

  const vda5050_core::master::PoseView view = agv->get_pose_view();
  if (view.source == vda5050_core::master::PoseSource::None)
  {
    response->status = GetPoseView::Response::AGV_SILENT;
    return;
  }

  response->status = GetPoseView::Response::SUCCESS;
  response->pose_view =
    build_pose_view_msg(view, request->manufacturer, request->serial_number);
}

}  // namespace vda5050_master_ros2
