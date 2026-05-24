#ifndef TIER4_AUTOWARE_API_HOST__ADAPTORS__PLANNING_FACTOR_CONVERT_HPP_
#define TIER4_AUTOWARE_API_HOST__ADAPTORS__PLANNING_FACTOR_CONVERT_HPP_

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <tier4_external_api_msgs/msg/planning_factor.hpp>

namespace tier4_autoware_api_host::adaptors
{

inline auto convert_behavior_type(const uint16_t & type)
{
  using InternalPlanningFactor = autoware_internal_planning_msgs::msg::PlanningFactor;
  using ExternalPlanningFactor = tier4_external_api_msgs::msg::PlanningFactor;

  switch (type) {
    case InternalPlanningFactor::NONE:
      return ExternalPlanningFactor::NONE;
    case InternalPlanningFactor::SLOW_DOWN:
      return ExternalPlanningFactor::SLOW_DOWN;
    case InternalPlanningFactor::STOP:
      return ExternalPlanningFactor::STOP;
    case InternalPlanningFactor::SHIFT_LEFT:
      return ExternalPlanningFactor::SHIFT_LEFT;
    case InternalPlanningFactor::SHIFT_RIGHT:
      return ExternalPlanningFactor::SHIFT_RIGHT;
    case InternalPlanningFactor::TURN_LEFT:
      return ExternalPlanningFactor::TURN_LEFT;
    case InternalPlanningFactor::TURN_RIGHT:
      return ExternalPlanningFactor::TURN_RIGHT;
    default:
      return ExternalPlanningFactor::UNKNOWN;
  }
}

inline tier4_external_api_msgs::msg::PlanningFactor convert(
  const std_msgs::msg::Header & header,
  const autoware_internal_planning_msgs::msg::PlanningFactor & internal)
{
  using InternalSafetyFactor = autoware_internal_planning_msgs::msg::SafetyFactor;
  using ExternalPlanningFactor = tier4_external_api_msgs::msg::PlanningFactor;
  using ExternalControlPoint = tier4_external_api_msgs::msg::PlanningFactorControlPoint;
  using ExternalObjectFactor = tier4_external_api_msgs::msg::DecisionFactorObject;
  using ExternalPointCloudFactor = tier4_external_api_msgs::msg::DecisionFactorPointCloud;

  ExternalPlanningFactor external;
  external.header = header;
  external.behavior_type = convert_behavior_type(internal.behavior);
  external.behavior_name = internal.module;
  external.behavior_detail = internal.detail;
  for (const autoware_internal_planning_msgs::msg::ControlPoint & control_point :
       internal.control_points) {
    ExternalControlPoint cp;
    cp.pose = control_point.pose;
    cp.distance = control_point.distance;
    external.control_points.push_back(cp);
  }
  external.is_safe = internal.safety_factors.is_safe;
  external.decision_detail = internal.safety_factors.detail;
  for (const auto & factor : internal.safety_factors.factors) {
    switch (factor.type) {
      case InternalSafetyFactor::OBJECT: {
        ExternalObjectFactor object_factor;
        object_factor.header = internal.safety_factors.header;
        object_factor.is_safe = factor.is_safe;
        object_factor.object_id = factor.object_id;
        if (factor.points.size() == 1) {
          object_factor.point = factor.points.front();
        }
        external.object_factors.push_back(object_factor);
        break;
      }
      case InternalSafetyFactor::POINTCLOUD: {
        ExternalPointCloudFactor pc_factor;
        pc_factor.header = internal.safety_factors.header;
        pc_factor.is_safe = factor.is_safe;
        pc_factor.points = factor.points;
        external.point_cloud_factors.push_back(pc_factor);
        break;
      }
    }
  }
  return external;
}

}  // namespace tier4_autoware_api_host::adaptors

#endif  // TIER4_AUTOWARE_API_HOST__ADAPTORS__PLANNING_FACTOR_CONVERT_HPP_
