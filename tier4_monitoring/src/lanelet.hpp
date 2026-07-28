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

#ifndef LANELET_HPP_
#define LANELET_HPP_

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <level4_mode_manager_msgs/msg/level4_availability.hpp>
#include <level4_mode_manager_msgs/msg/level4_driving_status.hpp>
#include <level4_mode_manager_msgs/srv/enable_level4_driving.hpp>
#include <level4_mode_manager_msgs/srv/override_level.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <array>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::level4_mode_manager
{
using autoware_adapi_v1_msgs::msg::MotionState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_adapi_v1_msgs::msg::Route;
using autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using level4_mode_manager_msgs::msg::Level4Availability;
using level4_mode_manager_msgs::msg::Level4DrivingStatus;
using level4_mode_manager_msgs::srv::EnableLevel4Driving;
using level4_mode_manager_msgs::srv::OverrideLevel;

class Level4ModeManager : public rclcpp::Node
{
public:
  explicit Level4ModeManager(const rclcpp::NodeOptions & options);

private:
  void lanelet_map_bin_callback(
    const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_map_msg);
  void operation_mode_state_callback(
    const OperationModeState::ConstSharedPtr input_operation_mode_state_msg);
  void route_callback(const Route::ConstSharedPtr input_route_msg);
  void call_select_level_and_run(
    const EnableLevel4Driving::Request::SharedPtr request,
    EnableLevel4Driving::Response::SharedPtr response);
  void call_override_level(
    const OverrideLevel::Request::SharedPtr request, OverrideLevel::Response::SharedPtr response);
  void on_timer();

  static std::optional<std::set<int64_t>> parse_level4_tag(const std::string & str);
  bool check_level4_availability(const Route & route);
  lanelet::ConstLanelets get_candidate_lanelets_from_pose(const geometry_msgs::msg::Pose & pose);

  /* Subscribers with essential callbacks */
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr lanelet_map_bin_sub_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_state_sub_;
  rclcpp::Subscription<Route>::SharedPtr route_sub_;

  /* Polling Subcribers */
  autoware_utils::InterProcessPollingSubscriber<MotionState> motion_state_sub_{
    this, "input/motion_state"};

  /* Publishers */
  rclcpp::Publisher<Level4Availability>::SharedPtr level4_availability_pub_;
  rclcpp::Publisher<Level4DrivingStatus>::SharedPtr level4_driving_status_pub_;

  /* Services */
  rclcpp::Service<EnableLevel4Driving>::SharedPtr enable_level4_driving_srv_;
  rclcpp::Service<OverrideLevel>::SharedPtr override_level_srv_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr change_operation_mode_cli_;

  /* Timer */
  rclcpp::TimerBase::SharedPtr timer_;

  /* Private Variables */
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  std::optional<OperationModeState> operation_mode_state_;
  std::optional<Route> route_;
  bool is_initialized_;
  bool is_level4_available_;
  bool is_level4_driving_;

  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture pending_future_;
  bool awaiting_response_;
  bool awaiting_autonomous_;
  bool latest_enable_level4_driving_;
  int response_waiting_counter_;

  static constexpr char level_defining_tag[] = "level4_operation_end_lanelet";
};
}  // namespace autoware::level4_mode_manager

#endif  // LANELET_HPP_
