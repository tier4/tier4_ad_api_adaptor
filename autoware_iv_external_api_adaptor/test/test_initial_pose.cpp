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
  void set_imu_calibrated(bool is_imu_calibrated) { is_imu_calibrated_ = is_imu_calibrated; }
  void set_auto_mode(bool is_auto_mode) { is_auto_mode_ = is_auto_mode; }
};

// サービスのテストがうまく行っていない。
TEST(TestInitialPose, DT_3_2)
{
  auto setInitializePoseTest =
    [](
      const tier4_external_api_msgs::srv::InitializePose::Request::SharedPtr request,
      const tier4_external_api_msgs::srv::InitializePose::Response::SharedPtr response) {
      RCLCPP_INFO(rclcpp::get_logger("test"), "[setInitializePose] Initialize pose service call!!");
      response->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
      response->status.message = "OK";
      RCLCPP_INFO(rclcpp::get_logger("test"), "[setInitializePose] Initialize pose service call");
    };

  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<InitialPoseTest>(node_options);
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  int count = 0;

  tier4_api_utils::ServiceProxyNodeInterface proxy(test_node.get());
  auto cli_set_initialize_pose_ = proxy.create_client<tier4_external_api_msgs::srv::InitializePose>(
    "/api/external/set/initialize_pose", rmw_qos_profile_services_default);
  auto srv_set_initialize_pose_ =
    proxy.create_service<tier4_external_api_msgs::srv::InitializePose>(
      "/api/autoware/set/initialize_pose", setInitializePoseTest, rmw_qos_profile_services_default);
  auto subscriber = test_node->create_subscription<sound_msgs::msg::SoundRequest>(
    "/localization/initial_pose/sound/request", rclcpp::QoS{1},
    [&count](const sound_msgs::msg::SoundRequest::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("test"), "Received message");
      count++;
    });
  tier4_external_api_msgs::srv::InitializePose::Request::SharedPtr request;
  request = std::make_shared<tier4_external_api_msgs::srv::InitializePose::Request>();
  request->pose.header.frame_id = "map";
  request->pose.header.stamp = rclcpp::Clock().now();
  request->pose.pose.pose.position.x = 0.0;  // X座標
  request->pose.pose.pose.position.y = 0.0;  // Y座標
  request->pose.pose.pose.position.z = 0.0;  // Z座標

  // クォータニオンで姿勢を設定
  request->pose.pose.pose.orientation.x = 0.0;
  request->pose.pose.pose.orientation.y = 0.0;
  request->pose.pose.pose.orientation.z = 0.0;
  request->pose.pose.pose.orientation.w = 1.0;  // 正面向き
  request->pose.pose.covariance.fill(0.0);
  auto initialize_pose_timeout = std::chrono::seconds(30);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);
  node->set_imu_calibrated(true);
  node->set_auto_mode(true);
  node->set_sound_locked(false);

  for (int i = 0; i < 30; i++) {
    if (i == 1) {
      executor.spin_some();
      std::thread service_thread([&]() {
        RCLCPP_INFO(
          rclcpp::get_logger("test"), "Initialize pose service call, request: %f, %f, %f",
          request->pose.pose.pose.position.x, request->pose.pose.pose.position.y,
          request->pose.pose.pose.position.z);
        const auto [status, response] =
          cli_set_initialize_pose_->call(request, initialize_pose_timeout);
        RCLCPP_INFO(
          rclcpp::get_logger("test"), "Initialize pose service call, status: %d", status.code);
        if (!response) {
          RCLCPP_ERROR(rclcpp::get_logger("test"), "Null response");
          return;
        }
        RCLCPP_INFO(
          rclcpp::get_logger("test"), "Initialize pose service call, response: %d",
          response->status.code);
      });
      service_thread.detach();
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  executor.spin_some();
  executor.spin_some();
  executor.spin_some();
  executor.spin_some();
  executor.spin_some();
  EXPECT_EQ(count, 1);
  rclcpp::shutdown();
}

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
