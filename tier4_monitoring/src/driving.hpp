// Copyright 2026 TIER IV, Inc.
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

#ifndef DRIVING_HPP_
#define DRIVING_HPP_

#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_external_api_msgs/msg/driving_status.hpp>
#include <tier4_external_api_msgs/srv/enable_driving.hpp>

namespace tier4_monitoring
{

class Driving
{
public:
  explicit Driving(rclcpp::Node & node);
  void publish(rclcpp::Time now);

private:
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using EnableDriving = tier4_external_api_msgs::srv::EnableDriving;
  using DrivingStatus = tier4_external_api_msgs::msg::DrivingStatus;
  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;

  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;

  rclcpp::Publisher<DrivingStatus>::SharedPtr pub_status_;
  rclcpp::Service<EnableDriving>::SharedPtr srv_enable_;

  void on_operation_mode(const OperationModeState & msg);
  void on_enable(
    const EnableDriving::Request::SharedPtr req, const EnableDriving::Response::SharedPtr res);

  DrivingLevel request_level_;
  OperationModeState operation_mode_;
  bool is_level2_available;
  bool is_level4_available;
};

}  // namespace tier4_monitoring

#endif  // DRIVING_HPP_
