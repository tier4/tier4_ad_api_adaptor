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

#ifndef VELOCITY_LIMIT_HPP_
#define VELOCITY_LIMIT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_external_api_msgs/msg/velocity_limit.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>

namespace tier4_autoware_api_extension
{
class VelocityLimit : public rclcpp::Node
{
public:
  explicit VelocityLimit(const rclcpp::NodeOptions & options);

private:
  using ExternalService = tier4_external_api_msgs::srv::SetVelocityLimit;
  using ExternalMessage = tier4_external_api_msgs::msg::VelocityLimit;
  using InternalMessage = autoware_internal_planning_msgs::msg::VelocityLimit;

  rclcpp::Service<ExternalService>::SharedPtr srv_api_velocity_;
  rclcpp::Publisher<ExternalMessage>::SharedPtr pub_api_velocity_;
  rclcpp::Publisher<InternalMessage>::SharedPtr pub_planning_velocity_;
  rclcpp::Subscription<InternalMessage>::SharedPtr sub_planning_velocity_;

  void on_velocity_limit_message(const InternalMessage::SharedPtr msg);
  void on_velocity_limit_service(
    const ExternalService::Request::SharedPtr request,
    const ExternalService::Response::SharedPtr response);

  void publishApiVelocity(double velocity);
  void publishPlanningVelocity(double velocity);
};

}  // namespace tier4_autoware_api_extension

#endif  // VELOCITY_LIMIT_HPP_
