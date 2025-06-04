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

#ifndef PLANNING_FACTOR_HPP_
#define PLANNING_FACTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <tier4_external_api_msgs/msg/planning_factor_array.hpp>

#include <vector>

namespace tier4_autoware_api_extension
{

using ExternalArray = tier4_external_api_msgs::msg::PlanningFactorArray;
using InternalArray = autoware_internal_planning_msgs::msg::PlanningFactorArray;

class PlanningFactor : public rclcpp::Node
{
public:
  explicit PlanningFactor(const rclcpp::NodeOptions & options);

private:
  void on_timer();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ExternalArray>::SharedPtr pub_planning_factors_;
  std::vector<rclcpp::Subscription<InternalArray>::SharedPtr> sub_planning_factors_;
  std::vector<InternalArray::ConstSharedPtr> factors_;

  double timeout_;
};

}  // namespace tier4_autoware_api_extension

#endif  // PLANNING_FACTOR_HPP_
