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

#include "test_initial_pose2.hpp"

constexpr auto initial_pose_timeout = std::chrono::seconds(10);

using std::placeholders::_1;
using std::placeholders::_2;

void InitialPoseSpy::setSoundLocked(bool is_sound_locked) { is_sound_locked_ = is_sound_locked; }
void InitialPoseSpy::setAutoMode(bool is_auto_mode) { is_auto_mode_ = is_auto_mode; }
void InitialPoseSpy::setImuCalibrated(bool is_imu_calibrated)
{
  is_imu_calibrated_ = is_imu_calibrated;
}

void TestInitialPose::SetUp()
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions node_options;
  node_ = std::make_shared<InitialPoseSpy>(node_options);
  test_node_ = std::make_shared<rclcpp::Node>("test_node");

  tier4_api_utils::ServiceProxyNodeInterface proxy(test_node_.get());
  cli_set_initialize_pose_ = proxy.create_client<InitializePose>(
    "/api/external/set/initialize_pose", rmw_qos_profile_services_default);

  sub_sound_request_ = test_node_->create_subscription<sound_msgs::msg::SoundRequest>(
    "localization/initial_pose/sound/request", rclcpp::QoS{1},
    [this](const sound_msgs::msg::SoundRequest::SharedPtr msg) {
      RCLCPP_INFO(test_node_->get_logger(), "soundRequestCallback: %s", msg->sound_type);
      node_->count_sound_request_++;
    });
  pub_sound_response_ = test_node_->create_publisher<tier4_external_api_msgs::msg::ResponseStatus>(
    "localization/initial_pose/sound/response", rclcpp::QoS{1});
  pub_operator_ = test_node_->create_publisher<tier4_external_api_msgs::msg::Operator>(
    "/api/external/get/operator", rclcpp::QoS{1});
  pub_imu_calibrated_ = test_node_->create_publisher<tier4_calibration_msgs::msg::BoolStamped>(
    "/sensing/imu/is_calibrated", rclcpp::QoS{1});
}

void TestInitialPose::TearDown() { rclcpp::shutdown(); }

TEST(TestInitialPose, DT_3_5)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<InitialPoseSpy>(node_options);
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_node");
}

TEST_F(TestInitialPose, DT_3_11)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(test_node_);

  tier4_external_api_msgs::srv::InitializePose::Request::SharedPtr request =
    std::make_shared<tier4_external_api_msgs::srv::InitializePose::Request>();
  node_->setSoundLocked(false);
  node_->setAutoMode(false);
  node_->setImuCalibrated(true);

  for (int i = 0; i < 300; ++i) {
    RCLCPP_INFO(
      test_node_->get_logger(), "request->pose.pose.pose.position.x: %f",
      request->pose.pose.pose.position.x);
    const auto [status, resp] = cli_set_initialize_pose_->call(request, initial_pose_timeout);
    RCLCPP_INFO(test_node_->get_logger(), "status: %d", status);
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO(test_node_->get_logger(), "count_sound_request_: %d", node_->count_sound_request_);
  }
  EXPECT_EQ(node_->count_sound_request_, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
