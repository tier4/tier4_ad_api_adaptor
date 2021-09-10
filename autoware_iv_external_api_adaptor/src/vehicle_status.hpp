// Copyright 2021 Tier IV, Inc.
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

#ifndef VEHICLE_STATUS_HPP_
#define VEHICLE_STATUS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_utils/autoware_api_utils.hpp"
#include "autoware_external_api_msgs/msg/vehicle_command_stamped.hpp"
#include "autoware_external_api_msgs/msg/vehicle_status_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"

namespace external_api
{

class VehicleStatus : public rclcpp::Node
{
public:
  explicit VehicleStatus(const rclcpp::NodeOptions & options);

private:
  // ros interface for vehicle status
  rclcpp::Publisher<autoware_external_api_msgs::msg::VehicleStatusStamped>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Steering>::SharedPtr sub_steering_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr sub_turn_signal_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr sub_gear_shift_;

  // ros interface for vehicle command
  rclcpp::Publisher<autoware_external_api_msgs::msg::VehicleCommandStamped>::SharedPtr pub_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr sub_cmd_;

  // ros callback
  void onTimer();

  // vehicle status
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;
  autoware_vehicle_msgs::msg::Steering::ConstSharedPtr steering_;
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr turn_signal_;
  autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr gear_shift_;
};

}  // namespace external_api

#endif  // VEHICLE_STATUS_HPP_
