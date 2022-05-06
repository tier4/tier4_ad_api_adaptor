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

#include "rtc_controller.hpp"

namespace external_api
{
RTCController::RTCController(const rclcpp::NodeOptions & options) : Node("external_api_rtc_controller", options)
{
  using namespace std::chrono_literals;
  using std::placeholders::_1;
  using std::placeholders::_2;
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);

  blind_spot_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/blind_spot/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::blindSpotCallback, this, _1));
  crosswalk_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/crosswalk/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::crosswalkCallback, this, _1));
  detection_area_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/detection_area/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::detectionAreaCallback, this, _1));
  intersection_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/intersect/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::intersectionCallback, this, _1));
  no_stopping_area_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/no_stopping_area/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::noStoppingAreaCallback, this, _1));
  occlusion_spot_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/occlusion_spot/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::occlusionSpotCallback, this, _1));
  stop_line_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/stop_line/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::stopLineCallback, this, _1));
  traffic_light_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/traffic_light/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::trafficLightCallback, this, _1));
  virtual_traffic_light_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/virtual_traffic_light/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::virtualTrafficLightCallback, this, _1));
  lane_change_left_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_left/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::laneChangeLeftCallback, this, _1));
  lane_change_right_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_right/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::laneChangeRightCallback, this, _1));
  avoidance_left_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_left/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::avoidanceLeftCallback, this, _1));
  avoidance_right_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_right/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::avoidanceRightCallback, this, _1));
  pull_over_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_over/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::pullOverCallback, this, _1));
  pull_out_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_out/cooperate_status",
    rclcpp::QoS(1).transient_local(), std::bind(&RTCController::pullOutCallback, this, _1));

  rtc_status_pub_ = create_publisher<CooperateStatusArray>(
    "/api/external/get/rtc_status", rclcpp::QoS(1).transient_local());

  timer_ = rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&RTCController::onTimer, this));
}

void RTCController::blindSpotCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    blind_spot_statuses_ = message->statuses;
  }
}

void RTCController::crosswalkCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    crosswalk_statuses_ = message->statuses;
  }
}

void RTCController::detectionAreaCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    detection_area_statuses_ = message->statuses;
  }
}

void RTCController::intersectionCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    intersection_statuses_ = message->statuses;
  }
}

void RTCController::noStoppingAreaCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    no_stopping_area_statuses_ = message->statuses;
  }
}

void RTCController::occlusionSpotCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    occlusion_spot_statuses_ = message->statuses;
  }
}

void RTCController::stopLineCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    stop_line_statuses_ = message->statuses;
  }
}

void RTCController::trafficLightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    traffic_light_statuses_ = message->statuses;
  }
}

void RTCController::virtualTrafficLightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    virtual_traffic_light_statuses_ = message->statuses;
  }
}

void RTCController::laneChangeLeftCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    lane_change_left_statuses_ = message->statuses;
  }
}

void RTCController::laneChangeRightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    lane_change_right_statuses_ = message->statuses;
  }
}

void RTCController::avoidanceLeftCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    avoidance_left_statuses_ = message->statuses;
  }
}

void RTCController::avoidanceRightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    avoidance_right_statuses_ = message->statuses;
  }
}

void RTCController::pullOverCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    pull_over_statuses_ = message->statuses;
  }
}

void RTCController::pullOutCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  if (!processing_statuses_){
    pull_out_statuses_ = message->statuses;
  }
}

void RTCController::insertionSort(std::vector<CooperateStatus> statuses_vector)
{
  tier4_rtc_msgs::msg::CooperateStatus current_status;
  for (size_t i = 1; i < statuses_vector.size(); i++)
  {
    current_status = statuses_vector[i];
    int j = i - 1;

    while (j >= 0 && current_status.distance < statuses_vector[j].distance)
    {
      statuses_vector[j + 1] = statuses_vector[j];
      j = j - 1;
    }
    statuses_vector[j + 1] = current_status;
  }
}

void RTCController::onTimer()
{
  processing_statuses_ = true;
  std::vector<CooperateStatus> cooperate_statuses;
  cooperate_statuses.insert(cooperate_statuses.end(), blind_spot_statuses_.begin(), blind_spot_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), crosswalk_statuses_.begin(), crosswalk_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), detection_area_statuses_.begin(), detection_area_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), intersection_statuses_.begin(), intersection_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), no_stopping_area_statuses_.begin(), no_stopping_area_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), occlusion_spot_statuses_.begin(), occlusion_spot_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), stop_line_statuses_.begin(), stop_line_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), traffic_light_statuses_.begin(), traffic_light_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), virtual_traffic_light_statuses_.begin(), virtual_traffic_light_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), lane_change_left_statuses_.begin(), lane_change_left_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), lane_change_right_statuses_.begin(), lane_change_right_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), avoidance_left_statuses_.begin(), avoidance_left_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), avoidance_right_statuses_.begin(), avoidance_right_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), pull_over_statuses_.begin(), pull_over_statuses_.end());
  cooperate_statuses.insert(cooperate_statuses.end(), pull_out_statuses_.begin(), pull_out_statuses_.end());

  insertionSort(cooperate_statuses);
  processing_statuses_ = false;
  CooperateStatusArray msg;
  msg.stamp = now();
  msg.statuses = cooperate_statuses;
  rtc_status_pub_->publish(msg);
}

}  // namespace external_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::RTCController)