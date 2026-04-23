// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LATERAL_OFFSET_HPP_
#define LATERAL_OFFSET_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/srv/set_lateral_offset.hpp>
#include <tier4_planning_msgs/srv/set_lateral_offset.hpp>

#include <memory>

namespace tier4_autoware_api_extension
{

class LateralOffset : public rclcpp::Node
{
public:
  explicit LateralOffset(const rclcpp::NodeOptions & options);

private:
  using ExternalService = tier4_external_api_msgs::srv::SetLateralOffset;
  using PlanningService = tier4_planning_msgs::srv::SetLateralOffset;

  rclcpp::Service<ExternalService>::SharedPtr srv_;
  rclcpp::Client<PlanningService>::SharedPtr cli_;

  void on_service(
    const std::shared_ptr<rmw_request_id_t> header,
    const ExternalService::Request::SharedPtr request);
};

}  // namespace tier4_autoware_api_extension

#endif  // LATERAL_OFFSET_HPP_
