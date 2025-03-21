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
: Node("external_api_initial_pose", options),
  is_playing_audio_(false),
  is_calibrated_(false),
  is_auto_key_(false)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);

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

  pub_sound_state_ = create_publisher<BoolStamped>(
    "/localization/initial_pose/audio_state",
    rclcpp::QoS(10));
  sub_state_sound_done_ = create_subscription<StateSoundDone>(
    "/autoware_state_machine/state_sound_done", 
    rclcpp::QoS(10),
    std::bind(&InitialPose::onStateSoundDone, this, _1));
    
  sub_operator_ = create_subscription<Operator>(
    "/api/external/get/operator", 
    rclcpp::QoS(10),
    std::bind(&InitialPose::onOperator, this, _1));
    
  sub_imu_calibrated_ = create_subscription<BoolStamped>(
    "/sensing/imu/is_calibrated", 
    rclcpp::QoS(10),
    std::bind(&InitialPose::onImuCalibrated, this, _1));
}

void InitialPose::setInitializePose(
  const tier4_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::InitializePose::Response::SharedPtr response)
{
  if (!is_playing_audio_) {
    if (!is_calibrated_ && is_auto_key_)
    {
      is_playing_audio_ = true;
    }
    BoolStamped msg;
    msg.data = is_playing_audio_;
    pub_sound_state_->publish(msg);
  }

  if(!is_calibrated_) {
    response->status.code = tier4_external_api_msgs::msg::ResponseStatus::ERROR;
    response->status.message = "ERROR: IMU calibration has not completed";
    return;
  }
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
  const auto [status, resp] = cli_set_initialize_pose_auto_->call(request, initial_pose_timeout);
  if (!tier4_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void InitialPose::onStateSoundDone(const StateSoundDone::SharedPtr msg)
{
  is_playing_audio_ = false;
}

void InitialPose::onOperator(const Operator::SharedPtr msg)
{
  is_auto_key_ = (msg->mode == tier4_external_api_msgs::msg::Operator::AUTONOMOUS);
}

void InitialPose::onImuCalibrated(const BoolStamped::SharedPtr msg)
{
  if (msg->data == true) {
    is_calibrated_ = true;
  }
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::InitialPose)
