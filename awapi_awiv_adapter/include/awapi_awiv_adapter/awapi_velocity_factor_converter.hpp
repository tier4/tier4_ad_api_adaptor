// Copyright 2024 TIER IV, Inc.
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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_VELOCITY_FACTOR_CONVERTER_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_VELOCITY_FACTOR_CONVERTER_HPP_

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/planning_behavior.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware_api
{

using autoware_adapi_v1_msgs::msg::PlanningBehavior;
using tier4_planning_msgs::msg::StopReason;

const std::map<std::string, std::string> conversion_map = {
  {PlanningBehavior::AVOIDANCE, StopReason::AVOIDANCE},
  {PlanningBehavior::CROSSWALK, StopReason::CROSSWALK},
  {PlanningBehavior::GOAL_PLANNER, StopReason::GOAL_PLANNER},
  {PlanningBehavior::INTERSECTION, StopReason::INTERSECTION},
  {PlanningBehavior::LANE_CHANGE, StopReason::LANE_CHANGE},
  {PlanningBehavior::MERGE, StopReason::MERGE_FROM_PRIVATE_ROAD},
  {PlanningBehavior::NO_DRIVABLE_LANE, StopReason::NO_DRIVABLE_LANE},
  {PlanningBehavior::NO_STOPPING_AREA, StopReason::NO_STOPPING_AREA},
  {PlanningBehavior::REAR_CHECK, StopReason::BLIND_SPOT},
  {PlanningBehavior::ROUTE_OBSTACLE, StopReason::OBSTACLE_STOP},
  {PlanningBehavior::SIDEWALK, StopReason::WALKWAY},
  {PlanningBehavior::START_PLANNER, StopReason::START_PLANNER},
  {PlanningBehavior::STOP_SIGN, StopReason::STOP_LINE},
  {PlanningBehavior::SURROUNDING_OBSTACLE, StopReason::SURROUND_OBSTACLE_CHECK},
  {PlanningBehavior::TRAFFIC_SIGNAL, StopReason::TRAFFIC_LIGHT},
  {PlanningBehavior::USER_DEFINED_DETECTION_AREA, StopReason::DETECTION_AREA},
  {PlanningBehavior::VIRTUAL_TRAFFIC_LIGHT, StopReason::VIRTUAL_TRAFFIC_LIGHT},
  {PlanningBehavior::RUN_OUT, StopReason::OBSTACLE_STOP},
  {PlanningBehavior::ADAPTIVE_CRUISE, "AdaptiveCruise"},
  {PlanningBehavior::ROUNDABOUT, StopReason::ROUNDABOUT}};
  

class AutowareIvVelocityFactorConverter
{
public:
  AutowareIvVelocityFactorConverter(rclcpp::Node & node, const double thresh_dist_to_stop_pose);

  tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr updateStopReasonArray(
    const autoware_adapi_v1_msgs::msg::VelocityFactorArray::ConstSharedPtr & msg_ptr);

private:
  tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr convert(
    const autoware_adapi_v1_msgs::msg::VelocityFactorArray::ConstSharedPtr & msg_ptr);

  tier4_planning_msgs::msg::StopFactor convert(
    const autoware_adapi_v1_msgs::msg::VelocityFactor & factor);

  rclcpp::Logger logger_;

  rclcpp::Clock::SharedPtr clock_;

  double thresh_dist_to_stop_pose_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_VELOCITY_FACTOR_CONVERTER_HPP_
