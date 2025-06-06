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

#include "planning_factor.hpp"

#include <string>
#include <vector>

namespace
{

using Header = std_msgs::msg::Header;
using InternalPlanningFactor = autoware_internal_planning_msgs::msg::PlanningFactor;
using InternalControlPoint = autoware_internal_planning_msgs::msg::ControlPoint;
using InternalSafetyFactor = autoware_internal_planning_msgs::msg::SafetyFactor;
using ExternalPlanningFactor = tier4_external_api_msgs::msg::PlanningFactor;
using ExternalControlPoint = tier4_external_api_msgs::msg::PlanningFactorControlPoint;
using ExternalObjectFactor = tier4_external_api_msgs::msg::DecisionFactorObject;
using ExternalPointCloudFactor = tier4_external_api_msgs::msg::DecisionFactorPointCloud;

auto convert_behavior_type(const uint16_t & type)
{
  // clang-format off
  switch (type) {
    case InternalPlanningFactor::NONE:        return ExternalPlanningFactor::NONE;
    case InternalPlanningFactor::SLOW_DOWN:   return ExternalPlanningFactor::SLOW_DOWN;
    case InternalPlanningFactor::STOP:        return ExternalPlanningFactor::STOP;
    case InternalPlanningFactor::SHIFT_LEFT:  return ExternalPlanningFactor::SHIFT_LEFT;
    case InternalPlanningFactor::SHIFT_RIGHT: return ExternalPlanningFactor::SHIFT_RIGHT;
    case InternalPlanningFactor::TURN_LEFT:   return ExternalPlanningFactor::TURN_LEFT;
    case InternalPlanningFactor::TURN_RIGHT:  return ExternalPlanningFactor::TURN_RIGHT;
    default:                                  return ExternalPlanningFactor::UNKNOWN;
  }
  // clang-format on
}

auto convert_control_point(const InternalControlPoint & internal)
{
  ExternalControlPoint external;
  external.pose = internal.pose;
  external.distance = internal.distance;
  return external;
}

auto convert_object_factor(const Header & header, const InternalSafetyFactor & factor)
{
  ExternalObjectFactor external;
  external.header = header;
  external.is_safe = factor.is_safe;
  external.object_id = factor.object_id;
  if (factor.points.size() == 1) {
    external.point = factor.points.front();
  }
  return external;
}

auto convert_point_cloud_factor(const Header & header, const InternalSafetyFactor & factor)
{
  ExternalPointCloudFactor external;
  external.header = header;
  external.is_safe = factor.is_safe;
  external.points = factor.points;
  return external;
}

auto convert(const Header & header, const InternalPlanningFactor & internal)
{
  ExternalPlanningFactor external;

  external.header = header;
  external.behavior_type = convert_behavior_type(internal.behavior);
  external.behavior_name = internal.module;
  external.behavior_detail = internal.detail;
  for (const auto & control_point : internal.control_points) {
    external.control_points.push_back(convert_control_point(control_point));
  }

  external.is_safe = internal.safety_factors.is_safe;
  external.decision_detail = internal.safety_factors.detail;
  for (const auto & factor : internal.safety_factors.factors) {
    switch (factor.type) {
      case InternalSafetyFactor::OBJECT:
        external.object_factors.push_back(
          convert_object_factor(internal.safety_factors.header, factor));
        break;
      case InternalSafetyFactor::POINTCLOUD:
        external.point_cloud_factors.push_back(
          convert_point_cloud_factor(internal.safety_factors.header, factor));
        break;
    }
  }

  return external;
}

}  // namespace

namespace tier4_autoware_api_extension
{

PlanningFactor::PlanningFactor(const rclcpp::NodeOptions & options)
: Node("planning_factor", options)
{
  timeout_ = declare_parameter<double>("timeout");

  const auto topics = declare_parameter<std::vector<std::string>>("topics");
  sub_planning_factors_.resize(topics.size());
  factors_.resize(topics.size());

  for (size_t i = 0; i < topics.size(); ++i) {
    const auto callback = [this](const int index) {
      return [this, index](const InternalArray::ConstSharedPtr msg) { factors_[index] = msg; };
    };
    sub_planning_factors_[i] = create_subscription<InternalArray>(topics[i], 1, callback(i));
  }

  pub_planning_factors_ = create_publisher<ExternalArray>("/api/external/get/planning_factors", 1);

  const auto period = rclcpp::Rate(declare_parameter<double>("rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void PlanningFactor::on_timer()
{
  // Remove timeout factors.
  for (auto & message : factors_) {
    if (message) {
      const auto duration = (now() - message->header.stamp).seconds();
      if (timeout_ < duration) {
        message.reset();
      }
    }
  }

  // Convert planning factors.
  ExternalArray external;
  external.stamp = now();
  for (const auto & message : factors_) {
    if (message) {
      for (const auto & factor : message->factors) {
        external.factors.push_back(convert(message->header, factor));
      }
    }
  }
  pub_planning_factors_->publish(external);
}

}  // namespace tier4_autoware_api_extension

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_autoware_api_extension::PlanningFactor)
