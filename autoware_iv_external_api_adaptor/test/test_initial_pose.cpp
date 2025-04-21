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

#include "initial_pose.hpp"

#include <gtest/gtest.h>

class InitialPoseTest : public external_api::InitialPose
{
public:
  using external_api::InitialPose::InitialPose;
  bool get_auto_mode() { return is_auto_mode_; }
  bool get_sound_locked() { return is_sound_locked_; }
  void set_sound_locked(bool is_sound_locked) { is_sound_locked_ = is_sound_locked; }
};

TEST(TestInitialPose, DT_3_5)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<InitialPoseTest>(node_options);
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  auto publisher = test_node->create_publisher<tier4_external_api_msgs::msg::Operator>(
    "/api/external/get/operator", rclcpp::QoS{1});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);
  tier4_external_api_msgs::msg::Operator operator_msg;
  operator_msg.mode = tier4_external_api_msgs::msg::Operator::AUTONOMOUS;
  publisher->publish(operator_msg);
  executor.spin_some();
  EXPECT_EQ(node->get_auto_mode(), true);
  rclcpp::shutdown();
}

TEST(TestInitialPose, DT_3_7)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<InitialPoseTest>(node_options);
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  auto publisher = test_node->create_publisher<tier4_external_api_msgs::msg::ResponseStatus>(
    "/localization/initial_pose/sound/response", rclcpp::QoS{1});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);
  node->set_sound_locked(true);
  tier4_external_api_msgs::msg::ResponseStatus response_status_msg;
  response_status_msg.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
  publisher->publish(response_status_msg);
  executor.spin_some();
  EXPECT_EQ(node->get_sound_locked(), false);
  rclcpp::shutdown();
}
