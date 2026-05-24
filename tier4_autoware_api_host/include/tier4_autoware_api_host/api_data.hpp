#ifndef TIER4_AUTOWARE_API_HOST__API_DATA_HPP_
#define TIER4_AUTOWARE_API_HOST__API_DATA_HPP_

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tier4_api_msgs/msg/door_status.hpp>
#include <tier4_api_msgs/msg/stop_command.hpp>
#include <tier4_api_msgs/msg/velocity_limit.hpp>
#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/msg/calibration_status.hpp>
#include <tier4_external_api_msgs/msg/cpu_temperature.hpp>
#include <tier4_external_api_msgs/msg/cpu_usage.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/msg/gpu_status.hpp>
#include <tier4_external_api_msgs/msg/hdd_status.hpp>
#include <tier4_external_api_msgs/msg/map_hash.hpp>
#include <tier4_external_api_msgs/msg/memory_status.hpp>
#include <tier4_external_api_msgs/msg/network_status.hpp>
#include <tier4_external_api_msgs/msg/observer.hpp>
#include <tier4_external_api_msgs/msg/operator.hpp>
#include <tier4_external_api_msgs/msg/rosbag_logging_mode.hpp>
#include <tier4_planning_msgs/msg/is_avoidance_possible.hpp>
#include <tier4_planning_msgs/msg/lane_change_status.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <tier4_rtc_msgs/msg/auto_mode_status.hpp>
#include <tier4_rtc_msgs/msg/auto_mode_status_array.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_system_msgs/msg/autoware_state.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <memory>

namespace tier4_autoware_api_host::plugin
{

struct ApiData
{
  // TF
  std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose;

  // AWAPI subscriptions (awapi_awiv_adapter)
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr steer;
  autoware_control_msgs::msg::Control::ConstSharedPtr vehicle_cmd;
  autoware_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr turn_indicators;
  autoware_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr hazard_lights;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr gear;
  tier4_vehicle_msgs::msg::BatteryStatus::ConstSharedPtr battery;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat;
  tier4_system_msgs::msg::AutowareState::ConstSharedPtr tier4_autoware_state;
  autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr control_mode;
  tier4_control_msgs::msg::GateMode::ConstSharedPtr gate_mode;
  autoware_adapi_v1_msgs::msg::MrmState::ConstSharedPtr mrm_state;
  autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr hazard_status;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diagnostics_agg;
  tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr stop_reason;
  tier4_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr lane_change_available;
  tier4_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr lane_change_ready;
  autoware_planning_msgs::msg::Path::ConstSharedPtr lane_change_candidate_path;
  tier4_planning_msgs::msg::IsAvoidancePossible::ConstSharedPtr obstacle_avoid_ready;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr obstacle_avoid_candidate_path;
  tier4_api_msgs::msg::VelocityLimit::ConstSharedPtr awapi_max_velocity;
  autoware_internal_planning_msgs::msg::VelocityLimit::ConstSharedPtr current_max_velocity;
  tier4_api_msgs::msg::StopCommand::ConstSharedPtr temporary_stop;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr autoware_trajectory;
  tier4_v2x_msgs::msg::InfrastructureCommandArray::ConstSharedPtr v2x_command;
  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr v2x_state;

  // External API subscriptions
  autoware_vehicle_msgs::msg::Engage::ConstSharedPtr engage_status;
  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state;
  tier4_external_api_msgs::msg::Emergency::ConstSharedPtr emergency;
  tier4_external_api_msgs::msg::Operator::ConstSharedPtr external_operator;
  tier4_external_api_msgs::msg::Observer::ConstSharedPtr external_observer;
  tier4_external_api_msgs::msg::RosbagLoggingMode::ConstSharedPtr rosbag_logging_mode;

  // External API — vehicle status subscriptions
  autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocity;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr steering;
  autoware_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr turn_indicators_status;
  autoware_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr hazard_lights_status;
  autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr gear_shift;

  // External API — door
  tier4_api_msgs::msg::DoorStatus::ConstSharedPtr door_status;

  // External API — Map
  tier4_external_api_msgs::msg::MapHash::ConstSharedPtr map_hash;

  // External API — SystemMonitor subscriptions
  tier4_external_api_msgs::msg::CpuTemperature::ConstSharedPtr cpu_temperature;
  tier4_external_api_msgs::msg::MemoryStatus::ConstSharedPtr memory_status;
  tier4_external_api_msgs::msg::GpuStatus::ConstSharedPtr gpu_status;
  tier4_external_api_msgs::msg::NetworkStatus::ConstSharedPtr network_status;
  tier4_external_api_msgs::msg::HddStatus::ConstSharedPtr hdd_status;

  // External API — cpu_usage
  tier4_external_api_msgs::msg::CpuUsage::ConstSharedPtr cpu_usage;

  // External API — calibration_status
  tier4_external_api_msgs::msg::CalibrationStatus::ConstSharedPtr accel_brake_map_status;

  // External API — localization_score
  autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr transform_probability;
  autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr nearest_voxel_transformation_likelihood;

  // External API — Route (AD API route)
  autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr adapi_route;

  // Internal API subscriptions
  tier4_control_msgs::msg::ExternalCommandSelectorMode::ConstSharedPtr external_select;
  autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr tracked_objects;

  // Extension subscriptions
  autoware_perception_msgs::msg::TrafficLightGroup::ConstSharedPtr traffic_light_group;
  autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr route_distance;

  // Extension — planning factors
  autoware_internal_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr planning_factors;

  // Deprecated engage (AD API operation mode)
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_state;

  // Internal operator state (latest from subscriptions)
  bool iv_msgs_emergency{false};
};

}  // namespace tier4_autoware_api_host::plugin

#endif  // TIER4_AUTOWARE_API_HOST__API_DATA_HPP_
