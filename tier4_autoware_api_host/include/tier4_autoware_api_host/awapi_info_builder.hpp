#ifndef TIER4_AUTOWARE_API_HOST__AWAPI_INFO_BUILDER_HPP_
#define TIER4_AUTOWARE_API_HOST__AWAPI_INFO_BUILDER_HPP_

#include "tier4_autoware_api_host/api_data.hpp"

#include <awapi_awiv_adapter/awapi_autoware_util.hpp>

namespace tier4_autoware_api_host
{

inline autoware_api::AutowareInfo build_awapi_info(const plugin::ApiData & data)
{
  autoware_api::AutowareInfo info;
  info.current_pose_ptr = data.current_pose;
  info.steer_ptr = data.steer;
  info.vehicle_cmd_ptr = data.vehicle_cmd;
  info.turn_indicators_ptr = data.turn_indicators;
  info.hazard_lights_ptr = data.hazard_lights;
  info.odometry_ptr = data.odometry;
  info.gear_ptr = data.gear;
  info.battery_ptr = data.battery;
  info.nav_sat_ptr = data.nav_sat;
  info.autoware_state_ptr = data.tier4_autoware_state;
  info.control_mode_ptr = data.control_mode;
  info.gate_mode_ptr = data.gate_mode;
  info.mrm_state_ptr = data.mrm_state;
  info.hazard_status_ptr = data.hazard_status;
  info.stop_reason_ptr = data.stop_reason;
  info.lane_change_available_ptr = data.lane_change_available;
  info.lane_change_ready_ptr = data.lane_change_ready;
  info.lane_change_candidate_ptr = data.lane_change_candidate_path;
  info.obstacle_avoid_ready_ptr = data.obstacle_avoid_ready;
  info.obstacle_avoid_candidate_ptr = data.obstacle_avoid_candidate_path;
  info.max_velocity_ptr = data.awapi_max_velocity;
  info.current_max_velocity_ptr = data.current_max_velocity;
  info.temporary_stop_ptr = data.temporary_stop;
  info.autoware_planning_traj_ptr = data.autoware_trajectory;
  info.v2x_command_ptr = data.v2x_command;
  info.v2x_state_ptr = data.v2x_state;
  info.diagnostic_ptr = data.diagnostics_agg;
  return info;
}

}  // namespace tier4_autoware_api_host

#endif  // TIER4_AUTOWARE_API_HOST__AWAPI_INFO_BUILDER_HPP_
