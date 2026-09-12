// Copyright 2021 TIER IV, Inc.
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

#include "velocity_limit.hpp"

#include <utility>

namespace tier4_autoware_api_extension
{
VelocityLimit::VelocityLimit(const rclcpp::NodeOptions & options) : Node("velocity_limit", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_api_velocity_ = create_service<ExternalService>(
    "/api/external/set/velocity_limit",
    std::bind(&VelocityLimit::on_velocity_limit_service, this, _1, _2));
  pub_api_velocity_ = create_publisher<ExternalMessage>(
    "/api/external/get/velocity_limit", rclcpp::QoS(1).transient_local());
  pub_planning_velocity_ = create_publisher<InternalMessage>(
    "/planning/scenario_planning/max_velocity_default", rclcpp::QoS(1).transient_local());
  sub_planning_velocity_ = create_subscription<InternalMessage>(
    "/planning/scenario_planning/current_max_velocity", rclcpp::QoS(1).transient_local(),
    std::bind(&VelocityLimit::on_velocity_limit_message, this, _1));
}

void VelocityLimit::on_velocity_limit_message(AUTOWARE_MESSAGE_CONST_SHARED_PTR(InternalMessage)
                                                msg)
{
  auto api = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_api_velocity_);
  api->stamp = msg->stamp;
  api->velocity = msg->max_velocity;
  pub_api_velocity_->publish(std::move(api));
}

void VelocityLimit::on_velocity_limit_service(
  AUTOWARE_SERVER_REQUEST_PTR(ExternalService) req,
  AUTOWARE_SERVER_RESPONSE_PTR(ExternalService) res)
{
  auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_planning_velocity_);
  msg->stamp = now();
  msg->max_velocity = req->velocity;
  pub_planning_velocity_->publish(std::move(msg));

  res->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
}

}  // namespace tier4_autoware_api_extension

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_autoware_api_extension::VelocityLimit)
