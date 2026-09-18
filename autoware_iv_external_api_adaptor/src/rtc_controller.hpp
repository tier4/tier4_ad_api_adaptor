// Copyright 2022 TIER IV, Inc.
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

#ifndef RTC_CONTROLLER_HPP_
#define RTC_CONTROLLER_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include "tier4_rtc_msgs/msg/auto_mode_status.hpp"
#include "tier4_rtc_msgs/msg/auto_mode_status_array.hpp"
#include "tier4_rtc_msgs/msg/cooperate_command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/srv/auto_mode.hpp"
#include "tier4_rtc_msgs/srv/auto_mode_with_module.hpp"
#include "tier4_rtc_msgs/srv/cooperate_commands.hpp"

#include <memory>
#include <string>
#include <vector>

using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;
using AutoMode = tier4_rtc_msgs::srv::AutoMode;
using AutoModeWithModule = tier4_rtc_msgs::srv::AutoModeWithModule;
using AutoModeStatusArray = tier4_rtc_msgs::msg::AutoModeStatusArray;
using AutoModeStatus = tier4_rtc_msgs::msg::AutoModeStatus;
using CooperateStatusArray = tier4_rtc_msgs::msg::CooperateStatusArray;
using CooperateStatus = tier4_rtc_msgs::msg::CooperateStatus;
using Module = tier4_rtc_msgs::msg::Module;
using RTCNodeT = autoware::agnocast_wrapper::Node;

class RTCModule
{
public:
  std::string cooperate_status_namespace_ = "/planning/cooperate_status";
  std::string cooperate_commands_namespace_ = "/planning/cooperate_commands";
  std::string auto_mode_status_namespace_ = "/planning/auto_mode_status";
  std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";
  std::vector<CooperateStatus> module_statuses_;
  AutoModeStatus auto_mode_status_;
  AUTOWARE_SUBSCRIPTION_PTR(CooperateStatusArray) module_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(AutoModeStatus) auto_mode_sub_;
  tier4_api_utils::Client<CooperateCommands, RTCNodeT>::SharedPtr cli_set_module_;
  tier4_api_utils::Client<AutoMode, RTCNodeT>::SharedPtr cli_set_auto_mode_;

  RTCModule(RTCNodeT * node, const std::string & name);
  void moduleCallback(const CooperateStatusArray::ConstSharedPtr message);
  void autoModeCallback(const AutoModeStatus::ConstSharedPtr message);
  void insertMessage(std::vector<CooperateStatus> & cooperate_statuses);
  void insertAutoModeMessage(std::vector<AutoModeStatus> & auto_mode_status);
  void callService(
    CooperateCommands::Request::SharedPtr request,
    const CooperateCommands::Response::SharedPtr & responses);
  void callAutoModeService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
};

namespace external_api
{
class RTCController : public autoware::agnocast_wrapper::Node
{
public:
  explicit RTCController(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<RTCModule> blind_spot_;
  std::unique_ptr<RTCModule> crosswalk_;
  std::unique_ptr<RTCModule> detection_area_;
  std::unique_ptr<RTCModule> intersection_;
  std::unique_ptr<RTCModule> intersection_occlusion_;
  std::unique_ptr<RTCModule> roundabout_;
  std::unique_ptr<RTCModule> no_stopping_area_;
  std::unique_ptr<RTCModule> occlusion_spot_;
  std::unique_ptr<RTCModule> traffic_light_;
  std::unique_ptr<RTCModule> virtual_traffic_light_;
  std::unique_ptr<RTCModule> lane_change_left_;
  std::unique_ptr<RTCModule> lane_change_right_;
  std::unique_ptr<RTCModule> ext_request_lane_change_left_;
  std::unique_ptr<RTCModule> ext_request_lane_change_right_;
  std::unique_ptr<RTCModule> avoidance_left_;
  std::unique_ptr<RTCModule> avoidance_right_;
  std::unique_ptr<RTCModule> avoidance_by_lc_left_;
  std::unique_ptr<RTCModule> avoidance_by_lc_right_;
  std::unique_ptr<RTCModule> goal_planner_;
  std::unique_ptr<RTCModule> start_planner_;

  /* publishers */
  AUTOWARE_PUBLISHER_PTR(CooperateStatusArray) rtc_status_pub_;
  AUTOWARE_PUBLISHER_PTR(AutoModeStatusArray) auto_mode_pub_;
  /* service from external */
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<CooperateCommands, RTCNodeT>::SharedPtr srv_set_rtc_;
  tier4_api_utils::Service<AutoModeWithModule, RTCNodeT>::SharedPtr srv_set_rtc_auto_mode_;

  /* Timer */
  AUTOWARE_TIMER_PTR timer_;
  AUTOWARE_TIMER_PTR auto_mode_timer_;

  void insertionSortAndValidation(std::vector<CooperateStatus> & statuses_vector);
  void checkInfDistance(CooperateStatus & status);

  void setRTC(
    const CooperateCommands::Request::SharedPtr requests,
    const CooperateCommands::Response::SharedPtr responses);
  void setRTCAutoMode(
    const AutoModeWithModule::Request::SharedPtr request,
    const AutoModeWithModule::Response::SharedPtr response);

  // ros callback
  void onTimer();
  void onAutoModeTimer();
};

}  // namespace external_api

#endif  // RTC_CONTROLLER_HPP_
