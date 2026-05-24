// Copyright 2022 TIER IV, Inc.
// Host-side RTC controller (from autoware_iv_external_api_adaptor rtc_controller).

#ifndef TIER4_AUTOWARE_API_HOST__ADAPTORS__RTC_CONTROLLER_HOST_HPP_
#define TIER4_AUTOWARE_API_HOST__ADAPTORS__RTC_CONTROLLER_HOST_HPP_

#include <tier4_api_utils/tier4_api_utils.hpp>

#include <tier4_rtc_msgs/msg/auto_mode_status.hpp>
#include <tier4_rtc_msgs/msg/auto_mode_status_array.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/msg/module.hpp>
#include <tier4_rtc_msgs/srv/auto_mode.hpp>
#include <tier4_rtc_msgs/srv/auto_mode_with_module.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>

#include <rclcpp/rclcpp.hpp>

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

class RTCModule
{
public:
  std::string cooperate_status_namespace_ = "/planning/cooperate_status";
  std::string cooperate_commands_namespace_ = "/planning/cooperate_commands";
  std::string auto_mode_status_namespace_ = "/planning/auto_mode_status";
  std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";
  std::vector<CooperateStatus> module_statuses_;
  AutoModeStatus auto_mode_status_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr module_sub_;
  rclcpp::Subscription<AutoModeStatus>::SharedPtr auto_mode_sub_;
  tier4_api_utils::Client<CooperateCommands>::SharedPtr cli_set_module_;
  tier4_api_utils::Client<AutoMode>::SharedPtr cli_set_auto_mode_;

  RTCModule(rclcpp::Node * node, const std::string & name);
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

namespace tier4_autoware_api_host::adaptors
{

class RtcControllerHost
{
public:
  explicit RtcControllerHost(rclcpp::Node * node);

  void on_timer();
  void on_auto_mode_timer();
  void set_rtc(
    const CooperateCommands::Request::SharedPtr requests,
    const CooperateCommands::Response::SharedPtr responses);
  void set_rtc_auto_mode(
    const AutoModeWithModule::Request::SharedPtr request,
    const AutoModeWithModule::Response::SharedPtr response);

  rclcpp::CallbackGroup::SharedPtr callback_group() const { return group_; }

private:
  void insertion_sort_and_validation(std::vector<CooperateStatus> & statuses_vector);
  static void check_inf_distance(CooperateStatus & status);

  rclcpp::Node * node_;
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

  rclcpp::Publisher<CooperateStatusArray>::SharedPtr rtc_status_pub_;
  rclcpp::Publisher<AutoModeStatusArray>::SharedPtr auto_mode_pub_;
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<CooperateCommands>::SharedPtr srv_set_rtc_;
  tier4_api_utils::Service<AutoModeWithModule>::SharedPtr srv_set_rtc_auto_mode_;
};

}  // namespace tier4_autoware_api_host::adaptors

#endif  // TIER4_AUTOWARE_API_HOST__ADAPTORS__RTC_CONTROLLER_HOST_HPP_
