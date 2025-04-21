// Copyright 2025 Tier IV, Inc.
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

#ifndef TEST_INITIAL_POSE2_HPP_
#define TEST_INITIAL_POSE2_HPP_

#include "initial_pose.hpp"

#include <gtest/gtest.h>

#include <memory>

using InitializePose = tier4_external_api_msgs::srv::InitializePose;

class InitialPoseSpy : public external_api::InitialPose
{
public:
  using external_api::InitialPose::InitialPose;
  void setSoundLocked(bool is_sound_locked);
  void setAutoMode(bool is_auto_mode);
  void setImuCalibrated(bool is_imu_calibrated);
  int count_sound_request_ = 0;
};

class TestInitialPose : public ::testing::Test
{
protected:
  std::shared_ptr<InitialPoseSpy> node_;
  rclcpp::Node::SharedPtr test_node_;
  tier4_api_utils::Client<InitializePose>::SharedPtr cli_set_initialize_pose_;

  rclcpp::Subscription<sound_msgs::msg::SoundRequest>::SharedPtr sub_sound_request_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::ResponseStatus>::SharedPtr pub_sound_response_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Operator>::SharedPtr pub_operator_;
  rclcpp::Publisher<tier4_calibration_msgs::msg::BoolStamped>::SharedPtr pub_imu_calibrated_;
  void SetUp() override;
  void TearDown() override;
};

#endif  // TEST_INITIAL_POSE2_HPP_
