// Copyright 2021 Tier IV, Inc.
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

#ifndef ROUTE_HPP_
#define ROUTE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_external_api_msgs/msg/route.hpp>
#include <tier4_external_api_msgs/srv/clear_route.hpp>
#include <tier4_external_api_msgs/srv/set_pose.hpp>
#include <tier4_external_api_msgs/srv/set_route.hpp>

namespace internal_api
{
class Route : public rclcpp::Node
{
public:
  explicit Route(const rclcpp::NodeOptions & options);

private:
  using ClearRoute = tier4_external_api_msgs::srv::ClearRoute;
  using SetRoute = tier4_external_api_msgs::srv::SetRoute;
  using SetPose = tier4_external_api_msgs::srv::SetPose;
  using HADMapRoute = autoware_auto_planning_msgs::msg::HADMapRoute;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<ClearRoute>::SharedPtr srv_clear_route_;
  tier4_api_utils::Service<SetRoute>::SharedPtr srv_set_route_;
  tier4_api_utils::Service<SetPose>::SharedPtr srv_set_goal_;
  tier4_api_utils::Service<SetPose>::SharedPtr srv_set_checkpoint_;
  tier4_api_utils::Client<std_srvs::srv::Trigger>::SharedPtr cli_clear_route_;
  rclcpp::Subscription<HADMapRoute>::SharedPtr sub_planning_route_;
  rclcpp::Publisher<HADMapRoute>::SharedPtr pub_planning_route_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_planning_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_planning_checkpoint_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Route>::SharedPtr pub_get_route_;

  // ros callback
  void clearRoute(
    const tier4_external_api_msgs::srv::ClearRoute::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::ClearRoute::Response::SharedPtr response);
  void setRoute(
    const tier4_external_api_msgs::srv::SetRoute::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetRoute::Response::SharedPtr response);
  void setGoal(
    const tier4_external_api_msgs::srv::SetPose::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetPose::Response::SharedPtr response);
  void setCheckpoint(
    const tier4_external_api_msgs::srv::SetPose::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetPose::Response::SharedPtr response);

  void onRoute(const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr message);
};

}  // namespace internal_api

#endif  // ROUTE_HPP_
