// Copyright 2025 TIER IV, Inc.
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

#ifndef MANUAL_STATUS_HPP_
#define MANUAL_STATUS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/vehicle_command_stamped.hpp>

namespace tier4_deprecated_api_adapter
{

using ExternalControlCommand = tier4_external_api_msgs::msg::ControlCommandStamped;
using ExternalVehicleCommand = tier4_external_api_msgs::msg::VehicleCommandStamped;

using InternalPedals = autoware_adapi_v1_msgs::msg::PedalsCommand;
using InternalSteering = autoware_adapi_v1_msgs::msg::SteeringCommand;
using InternalControl = autoware_control_msgs::msg::Control;

class ManualStatus : public rclcpp::Node
{
public:
  explicit ManualStatus(const rclcpp::NodeOptions & options);

private:
  // Status input.
  rclcpp::Subscription<InternalControl>::SharedPtr sub_control_command_;
  rclcpp::Subscription<InternalPedals>::SharedPtr sub_selected_pedals_;
  rclcpp::Subscription<InternalSteering>::SharedPtr sub_selected_steering_;
  float steering_tire_angle_ = 0.0f;
  float steering_tire_velocity_ = 0.0f;

  // Status output.
  rclcpp::Publisher<ExternalVehicleCommand>::SharedPtr pub_vehicle_;  // velocity and accel
  rclcpp::Publisher<ExternalControlCommand>::SharedPtr pub_control_;  // steering and pedals

  // Callbacks.
  void relay_control_command(const InternalControl & msg);
  void relay_pedals(const InternalPedals & msg);
  void relay_steering(const InternalSteering & msg);
};

}  // namespace tier4_deprecated_api_adapter

#endif  // MANUAL_STATUS_HPP_
