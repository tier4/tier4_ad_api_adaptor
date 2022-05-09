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

#include <memory>

namespace external_api
{
RTCController::RTCController(const rclcpp::NodeOptions & options)
: Node("external_api_rtc_controller", options)
{
  using namespace std::literals::chrono_literals;
  using std::placeholders::_1;
  using std::placeholders::_2;
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);

  blind_spot_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/blind_spot/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::blindSpotCallback, this, _1));
  crosswalk_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/crosswalk/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::crosswalkCallback, this, _1));
  detection_area_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/detection_area/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::detectionAreaCallback, this, _1));
  intersection_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/intersect/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::intersectionCallback, this, _1));
  no_stopping_area_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/no_stopping_area/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::noStoppingAreaCallback, this, _1));
  occlusion_spot_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/occlusion_spot/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::occlusionSpotCallback, this, _1));
  traffic_light_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/traffic_light/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::trafficLightCallback, this, _1));
  virtual_traffic_light_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE +
      "/behavior_velocity_planner/virtual_traffic_light/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::virtualTrafficLightCallback, this, _1));
  lane_change_left_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_left/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::laneChangeLeftCallback, this, _1));
  lane_change_right_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_right/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::laneChangeRightCallback, this, _1));
  avoidance_left_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_left/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::avoidanceLeftCallback, this, _1));
  avoidance_right_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_right/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::avoidanceRightCallback, this, _1));
  pull_over_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_over/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::pullOverCallback, this, _1));
  pull_out_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_out/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCController::pullOutCallback, this, _1));

  rtc_status_pub_ =
    create_publisher<CooperateStatusArray>("/api/external/get/rtc_status", rclcpp::QoS(1));

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_rtc_ = proxy.create_service<CooperateCommands>(
    "/api/external/set/rtc_commands", std::bind(&RTCController::setRTC, this, _1, _2),
    rmw_qos_profile_services_default, group_);

  cli_set_blind_spot_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/blind_spot/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_crosswalk_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/crosswalk/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_detection_area_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/detection_area/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_intersection_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/intersect/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_no_stopping_area_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/no_stopping_area/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_occlusion_spot_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/occlusion_spot/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_traffic_light_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/traffic_light/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_virtual_traffic_light_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE +
      "/behavior_velocity_planner/virtual_traffic_light/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_lane_change_left_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_left/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_lane_change_right_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_right/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_avoidance_left_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_left/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_avoidance_right_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_right/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_pull_over_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_over/cooperate_commands",
    rmw_qos_profile_services_default);
  cli_set_pull_out_ = proxy.create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_out/cooperate_commands",
    rmw_qos_profile_services_default);

  timer_ = rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&RTCController::onTimer, this));
}

void RTCController::blindSpotCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  blind_spot_statuses_ = message->statuses;
}

void RTCController::crosswalkCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  crosswalk_statuses_ = message->statuses;
}

void RTCController::detectionAreaCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  detection_area_statuses_ = message->statuses;
}

void RTCController::intersectionCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  intersection_statuses_ = message->statuses;
}

void RTCController::noStoppingAreaCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  no_stopping_area_statuses_ = message->statuses;
}

void RTCController::occlusionSpotCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  occlusion_spot_statuses_ = message->statuses;
}

void RTCController::trafficLightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  traffic_light_statuses_ = message->statuses;
}

void RTCController::virtualTrafficLightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  virtual_traffic_light_statuses_ = message->statuses;
}

void RTCController::laneChangeLeftCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  lane_change_left_statuses_ = message->statuses;
}

void RTCController::laneChangeRightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  lane_change_right_statuses_ = message->statuses;
}

void RTCController::avoidanceLeftCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  avoidance_left_statuses_ = message->statuses;
}

void RTCController::avoidanceRightCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  avoidance_right_statuses_ = message->statuses;
}

void RTCController::pullOverCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  pull_over_statuses_ = message->statuses;
}

void RTCController::pullOutCallback(const CooperateStatusArray::ConstSharedPtr message)
{
  pull_out_statuses_ = message->statuses;
}

void RTCController::insertionSort(std::vector<CooperateStatus> statuses_vector)
{
  tier4_rtc_msgs::msg::CooperateStatus current_status;
  for (size_t i = 1; i < statuses_vector.size(); i++) {
    current_status = statuses_vector[i];
    int j = i - 1;

    while (j >= 0 && current_status.distance < statuses_vector[j].distance) {
      statuses_vector[j + 1] = statuses_vector[j];
      j = j - 1;
    }
    statuses_vector[j + 1] = current_status;
  }
}

void RTCController::onTimer()
{
  std::vector<CooperateStatus> cooperate_statuses;
  cooperate_statuses.insert(
    cooperate_statuses.end(), blind_spot_statuses_.begin(), blind_spot_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), crosswalk_statuses_.begin(), crosswalk_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), detection_area_statuses_.begin(), detection_area_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), intersection_statuses_.begin(), intersection_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), no_stopping_area_statuses_.begin(), no_stopping_area_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), occlusion_spot_statuses_.begin(), occlusion_spot_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), traffic_light_statuses_.begin(), traffic_light_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), virtual_traffic_light_statuses_.begin(),
    virtual_traffic_light_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), lane_change_left_statuses_.begin(), lane_change_left_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), lane_change_right_statuses_.begin(),
    lane_change_right_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), avoidance_left_statuses_.begin(), avoidance_left_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), avoidance_right_statuses_.begin(), avoidance_right_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), pull_over_statuses_.begin(), pull_over_statuses_.end());
  cooperate_statuses.insert(
    cooperate_statuses.end(), pull_out_statuses_.begin(), pull_out_statuses_.end());

  insertionSort(cooperate_statuses);
  CooperateStatusArray msg;
  msg.stamp = now();
  msg.statuses = cooperate_statuses;
  rtc_status_pub_->publish(msg);
}

void RTCController::setRTC(
  const CooperateCommands::Request::SharedPtr requests,
  const CooperateCommands::Response::SharedPtr responses)
{
  for (tier4_rtc_msgs::msg::CooperateCommand & command : requests->commands) {
    auto request = std::make_shared<CooperateCommands::Request>();
    request->commands = {command};
    switch (command.module.type) {
      case Module::LANE_CHANGE_LEFT: {
        const auto [status, resp] = cli_set_lane_change_left_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::LANE_CHANGE_RIGHT: {
        const auto [status, resp] = cli_set_lane_change_right_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::AVOIDANCE_LEFT: {
        const auto [status, resp] = cli_set_avoidance_left_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::AVOIDANCE_RIGHT: {
        const auto [status, resp] = cli_set_avoidance_right_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::PULL_OVER: {
        const auto [status, resp] = cli_set_pull_over_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::PULL_OUT: {
        const auto [status, resp] = cli_set_pull_out_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::TRAFFIC_LIGHT: {
        const auto [status, resp] = cli_set_traffic_light_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::INTERSECTION: {
        const auto [status, resp] = cli_set_intersection_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::CROSSWALK: {
        const auto [status, resp] = cli_set_crosswalk_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::BLIND_SPOT: {
        const auto [status, resp] = cli_set_blind_spot_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::DETECTION_AREA: {
        const auto [status, resp] = cli_set_detection_area_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::NO_STOPPING_AREA: {
        const auto [status, resp] = cli_set_no_stopping_area_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
      case Module::OCCLUSION_SPOT: {
        const auto [status, resp] = cli_set_occlusion_spot_->call(request);
        responses->responses.insert(
          responses->responses.end(), resp->responses.begin(), resp->responses.end());
        break;
      }
        // virtual_traffic not found
    }
  }
}

}  // namespace external_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::RTCController)
