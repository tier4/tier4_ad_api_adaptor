// Copyright 2021 TIER IV, Inc.
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

#include "initial_pose.hpp"

#include <memory>

namespace external_api
{

constexpr auto initial_pose_timeout = std::chrono::seconds(300);

InitialPose::InitialPose(const rclcpp::NodeOptions & options)
: Node("external_api_initial_pose", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);
  is_sound_locked_ = false;
  is_imu_calibrated_ = false;
  is_auto_mode_ = false;

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_initialize_pose_ = proxy.create_service<InitializePose>(
    "/api/external/set/initialize_pose", std::bind(&InitialPose::setInitializePose, this, _1, _2),
    rmw_qos_profile_services_default, group_);
  srv_set_initialize_pose_auto_ = proxy.create_service<InitializePoseAuto>(
    "/api/external/set/initialize_pose_auto",
    std::bind(&InitialPose::setInitializePoseAuto, this, _1, _2), rmw_qos_profile_services_default,
    group_);
  cli_set_initialize_pose_ = proxy.create_client<InitializePose>(
    "/api/autoware/set/initialize_pose", rmw_qos_profile_services_default);
  cli_set_initialize_pose_auto_ = proxy.create_client<InitializePoseAuto>(
    "/api/autoware/set/initialize_pose_auto", rmw_qos_profile_services_default);

  pub_sound_request_ = create_publisher<sound_msgs::msg::SoundRequest>(
    "localization/initial_pose/sound/request", rclcpp::QoS{1});
  sub_sound_response_ = create_subscription<tier4_external_api_msgs::msg::ResponseStatus>(
    "/localization/initial_pose/sound/response", rclcpp::QoS{1},
    std::bind(&InitialPose::soundResponseCallback, this, std::placeholders::_1));
  sub_operator_ = create_subscription<tier4_external_api_msgs::msg::Operator>(
    "/api/external/get/operator", rclcpp::QoS{1},
    std::bind(&InitialPose::operatorCallback, this, std::placeholders::_1));
  sub_imu_calibrated_ = create_subscription<tier4_calibration_msgs::msg::BoolStamped>(
    "/sensing/imu/is_calibrated", rclcpp::QoS{1},
    std::bind(&InitialPose::imuCalibratedCallback, this, std::placeholders::_1));
}

void InitialPose::setInitializePose(
  const tier4_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::InitializePose::Response::SharedPtr response)
{
  const auto [status, resp] = cli_set_initialize_pose_->call(request, initial_pose_timeout);
  if (!tier4_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void InitialPose::setInitializePoseAuto(
  const tier4_external_api_msgs::srv::InitializePoseAuto::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::InitializePoseAuto::Response::SharedPtr response)
{
  if (is_imu_calibrated_) {
    const auto [status, resp] = cli_set_initialize_pose_auto_->call(request, initial_pose_timeout);
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  } else {
    response->status.code = tier4_external_api_msgs::msg::ResponseStatus::ERROR;
    response->status.message = "ERROR";
    is_sound_locked_ = canPlaySound(is_sound_locked_, is_auto_mode_);
    sound_msgs::msg::SoundRequest sound_req;
    sound_req.stamp = this->now();
    sound_req.sound_type = "alert_imu_initialize";
    pub_sound_request_->publish(sound_req);
    RCLCPP_WARN(get_logger(), "WARN: IMU calibration incomplete. - Localization request rejected.");
  }
}

bool InitialPose::canPlaySound(bool is_sound_locked, bool is_auto_mode)
{
  if (!is_sound_locked_) {
    return false;
  }
  if (is_auto_mode_) {
    return false;
  }
  return true;
}

void InitialPose::imuCalibratedCallback(
  const tier4_calibration_msgs::msg::BoolStamped::ConstSharedPtr msg)
{
  is_imu_calibrated_ = msg->data;
}

void InitialPose::operatorCallback(const tier4_external_api_msgs::msg::Operator::ConstSharedPtr msg)
{
  if (msg->mode == tier4_external_api_msgs::msg::Operator::AUTONOMOUS) {
    is_auto_mode_ = true;
  } else {
    is_auto_mode_ = false;
  }
}

void InitialPose::soundResponseCallback(
  const tier4_external_api_msgs::msg::ResponseStatus::ConstSharedPtr msg)
{
  is_sound_locked_ = false;
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::InitialPose)
