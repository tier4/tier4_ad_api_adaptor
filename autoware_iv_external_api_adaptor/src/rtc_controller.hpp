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

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/srv/cooperate_commands.hpp"

namespace external_api
{
class RTCController : public rclcpp::Node
{
public:
  explicit RTCController(const rclcpp::NodeOptions & options);

private:
  using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;
  using CooperateStatusArray = tier4_rtc_msgs::msg::CooperateStatusArray;
  using CooperateStatus = tier4_rtc_msgs::msg::CooperateStatus;

  std::vector<CooperateStatus> blind_spot_statuses_;
  std::vector<CooperateStatus> crosswalk_statuses_;
  std::vector<CooperateStatus> detection_area_statuses_;
  std::vector<CooperateStatus> intersection_statuses_;
  std::vector<CooperateStatus> no_stopping_area_statuses_;
  std::vector<CooperateStatus> occlusion_spot_statuses_;
  std::vector<CooperateStatus> stop_line_statuses_;
  std::vector<CooperateStatus> traffic_light_statuses_;
  std::vector<CooperateStatus> virtual_traffic_light_statuses_;
  std::vector<CooperateStatus> lane_change_left_statuses_;
  std::vector<CooperateStatus> lane_change_right_statuses_;
  std::vector<CooperateStatus> avoidance_left_statuses_;
  std::vector<CooperateStatus> avoidance_right_statuses_;
  std::vector<CooperateStatus> pull_over_statuses_;
  std::vector<CooperateStatus> pull_out_statuses_;

  bool processing_statuses_ = false;

  std::string BEHAVIOR_PLANNING_NAMESPACE = "/planning/scenario_planning/lane_driving/behavior_planning";

  /* subscribers */
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr blind_spot_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr crosswalk_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr detection_area_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr intersection_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr no_stopping_area_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr occlusion_spot_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr stop_line_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr traffic_light_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr virtual_traffic_light_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr lane_change_left_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr lane_change_right_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr avoidance_left_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr avoidance_right_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr pull_over_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr pull_out_sub_;

  /* publishers */
  rclcpp::Publisher<CooperateStatusArray>::SharedPtr rtc_status_pub_;

  /* Timer */
  rclcpp::TimerBase::SharedPtr timer_;

  void insertionSort(std::vector<CooperateStatus> statuses_vector);

  void blindSpotCallback(const CooperateStatusArray::ConstSharedPtr message);
  void crosswalkCallback(const CooperateStatusArray::ConstSharedPtr message);
  void detectionAreaCallback(const CooperateStatusArray::ConstSharedPtr message);
  void intersectionCallback(const CooperateStatusArray::ConstSharedPtr message);
  void noStoppingAreaCallback(const CooperateStatusArray::ConstSharedPtr message);
  void occlusionSpotCallback(const CooperateStatusArray::ConstSharedPtr message);
  void stopLineCallback(const CooperateStatusArray::ConstSharedPtr message);
  void trafficLightCallback(const CooperateStatusArray::ConstSharedPtr message);
  void virtualTrafficLightCallback(const CooperateStatusArray::ConstSharedPtr message);
  void laneChangeLeftCallback(const CooperateStatusArray::ConstSharedPtr message);
  void laneChangeRightCallback(const CooperateStatusArray::ConstSharedPtr message);
  void avoidanceLeftCallback(const CooperateStatusArray::ConstSharedPtr message);
  void avoidanceRightCallback(const CooperateStatusArray::ConstSharedPtr message);
  void pullOverCallback(const CooperateStatusArray::ConstSharedPtr message);
  void pullOutCallback(const CooperateStatusArray::ConstSharedPtr message);

  // ros callback
  void onTimer();
};

}  // namespace external_api

#endif  // RTC_CONTROLLER_HPP_