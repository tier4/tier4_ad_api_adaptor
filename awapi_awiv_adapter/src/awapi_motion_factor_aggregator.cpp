// Copyright 2020 TIER IV, Inc.
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

#include "awapi_awiv_adapter/awapi_motion_factor_aggregator.hpp"

#include <memory>
#include <vector>

namespace autoware_api
{
AutowareIvMotionFactorAggregator::AutowareIvMotionFactorAggregator(rclcpp::Node & node)
: logger_(node.get_logger().get_child("awapi_awiv_motion_factor_aggregator")),
  clock_(node.get_clock()),
{
}

void AutowareIvMotionFactorAggregator::updateSceneModuleMotionFactorArray(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
{
  if(!scene_module_factor_.empty())
  {
    scene_module_factor_.clear();
  }
  scene_module_factor_ = *msg_ptr;
}

void AutowareIvMotionFactorAggregator::updateObstacleStopMotionFactorArray(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
{
  if(!obstacle_stop_factor_.empty())
  {
    obstacle_stop_factor_.clear();
  }
  obstacle_stop_factor_ = *msg_ptr;
}

void AutowareIvMotionFactorAggregator::updateSurroundObstacleMotionFactorArray(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
{
  if(!surround_obstacle_factor_.empty())
  {
    surround_obstacle_factor_.clear();
  }
  surround_obstacle_factor_ = *msg_ptr;
}

tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr
AutowareIvMotionFactorAggregator::makeMotionFactorArray(const AutowareInfo & aw_info)
{
  tier4_planning_msgs::msg::MotionFactorArray motion_factor_array_msg;
  // input header
  motion_factor_array_msg.header.frame_id = "map";
  motion_factor_array_msg.header.stamp = clock_->now();

  // input motion factor
  for (const auto & motion_factor : scene_module_factor.motion_factors) {
    appendMotionFactorToArray(motion_factor, &motion_factor_array_msg, aw_info);
  }

  for (const auto & motion_factor : obstacle_factor.motion_factors) {
    appendMotionFactorToArray(motion_factor, &motion_factor_array_msg, aw_info);
  }

  for (const auto & motion_factor : surround_obstacle_factor.motion_factors) {
    appendMotionFactorToArray(motion_factor, &motion_factor_array_msg, aw_info);
  }

  // sort factors in ascending order
  std::sort(motion_factor_array_msg.begin(), motion_factor_array_msg.end(), compareFactorsByDistance);

  return std::make_shared<tier4_planning_msgs::msg::MotionFactorArray>(motion_factor_array_msg);
}

static bool AutowareIvMotionFactorAggregator::compareFactorsByDistance(
  tier4_planning_msgs::msg::MotionFactor &a, tier4_planning_msgs::msg::MotionFactor &b)
{
  return a.dist_to_stop_pose < b.dist_to_stop_pose;
}

void AutowareIvMotionFactorAggregator::appendMotionFactorToArray(
  const tier4_planning_msgs::msg::MotionFactor & motion_factor,
  tier4_planning_msgs::msg::MotionFactorArray * motion_factor_array, const AutowareInfo & aw_info)
{
  // calculate dist_to_stop_pose
  const auto motion_factor_with_dist = inputStopDistToMotionFactor(motion_factor, aw_info);

  // if not exist same reason msg, append new stop reason
  motion_factor_array->motion_factors.emplace_back(motion_factor_with_dist);
}

tier4_planning_msgs::msg::MotionFactor AutowareIvMotionFactorAggregator::inputStopDistToMotionFactor(
  const tier4_planning_msgs::msg::MotionFactor & motion_factor, const AutowareInfo & aw_info)
{
  if (!aw_info.autoware_planning_traj_ptr || !aw_info.current_pose_ptr) {
    // pass through all stop reason
    return motion_factor;
  }

  auto motion_factor_with_dist = motion_factor;
  const auto & trajectory = *aw_info.autoware_planning_traj_ptr;
  const auto & current_pose = aw_info.current_pose_ptr->pose;
  motion_factor.dist_to_stop_pose =
    planning_util::calcDistanceAlongTrajectory(trajectory, current_pose, motion_factor.stop_pose);
  
  return motion_factor_with_dist;
}

}  // namespace autoware_api
