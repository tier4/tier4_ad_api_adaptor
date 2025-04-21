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

#ifndef INITIAL_POSE_HPP_
#define INITIAL_POSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tier4_api_utils/tier4_api_utils.hpp"

#include "sound_msgs/msg/sound_request.hpp"
#include "tier4_calibration_msgs/msg/bool_stamped.hpp"
#include "tier4_external_api_msgs/msg/operator.hpp"
#include "tier4_external_api_msgs/msg/response_status.hpp"
#include "tier4_external_api_msgs/srv/initialize_pose.hpp"
#include "tier4_external_api_msgs/srv/initialize_pose_auto.hpp"

namespace external_api
{

class InitialPose : public rclcpp::Node
{
public:
  explicit InitialPose(const rclcpp::NodeOptions & options);

private:
  using InitializePose = tier4_external_api_msgs::srv::InitializePose;
  using InitializePoseAuto = tier4_external_api_msgs::srv::InitializePoseAuto;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<InitializePose>::SharedPtr srv_set_initialize_pose_;
  tier4_api_utils::Service<InitializePoseAuto>::SharedPtr srv_set_initialize_pose_auto_;
  tier4_api_utils::Client<InitializePose>::SharedPtr cli_set_initialize_pose_;
  tier4_api_utils::Client<InitializePoseAuto>::SharedPtr cli_set_initialize_pose_auto_;

  rclcpp::Publisher<sound_msgs::msg::SoundRequest>::SharedPtr pub_sound_request_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::ResponseStatus>::SharedPtr sub_sound_response_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::Operator>::SharedPtr sub_operator_;
  rclcpp::Subscription<tier4_calibration_msgs::msg::BoolStamped>::SharedPtr sub_imu_calibrated_;

  // ros callback
  void setInitializePose(
    const tier4_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::InitializePose::Response::SharedPtr response);
  void setInitializePoseAuto(
    const tier4_external_api_msgs::srv::InitializePoseAuto::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::InitializePoseAuto::Response::SharedPtr response);
  void imuCalibratedCallback(const tier4_calibration_msgs::msg::BoolStamped::ConstSharedPtr msg);
  void operatorCallback(const tier4_external_api_msgs::msg::Operator::ConstSharedPtr msg);
  void soundResponseCallback(
    const tier4_external_api_msgs::msg::ResponseStatus::ConstSharedPtr msg);

  bool canPlaySound(bool is_sound_locked, bool is_auto_mode);

protected:
  bool is_sound_locked_;
  bool is_imu_calibrated_;
  bool is_auto_mode_;
};

}  // namespace external_api

#endif  // INITIAL_POSE_HPP_
