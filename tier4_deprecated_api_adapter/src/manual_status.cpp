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

#include "manual_status.hpp"

namespace tier4_deprecated_api_adapter
{

ManualStatus::ManualStatus(const rclcpp::NodeOptions & options) : Node("manual_status", options)
{
  const auto bind = [this](auto && func) { return std::bind(func, this, std::placeholders::_1); };

  sub_control_command_ = create_subscription<InternalControl>(
    "/external/selected/control_cmd", 1, bind(&ManualStatus::relay_control_command));
  sub_selected_pedals_ = create_subscription<InternalPedals>(
    "/external/selected/pedals_cmd", 1, bind(&ManualStatus::relay_pedals));
  sub_selected_steering_ = create_subscription<InternalSteering>(
    "/external/selected/steering_cmd", 1, bind(&ManualStatus::relay_steering));

  pub_vehicle_ =
    create_publisher<ExternalVehicleCommand>("/api/external/get/command/selected/vehicle", 1);
  pub_control_ =
    create_publisher<ExternalControlCommand>("/api/external/get/command/selected/control", 1);
}

void ManualStatus::relay_control_command(const InternalControl & msg)
{
  // Note that the internal control command corresponds to the external vehicle command.
  ExternalVehicleCommand command;
  command.stamp = msg.stamp;
  command.command.velocity = msg.longitudinal.velocity;
  command.command.acceleration = msg.longitudinal.acceleration;
  pub_vehicle_->publish(command);
}

void ManualStatus::relay_pedals(const InternalPedals & msg)
{
  ExternalControlCommand command;
  command.stamp = msg.stamp;
  command.control.throttle = msg.throttle;
  command.control.brake = msg.brake;
  command.control.steering_angle = steering_tire_angle_;
  command.control.steering_angle_velocity = steering_tire_velocity_;
  pub_control_->publish(command);
}

void ManualStatus::relay_steering(const InternalSteering & msg)
{
  steering_tire_angle_ = msg.steering_tire_angle;
  steering_tire_velocity_ = msg.steering_tire_velocity;
}

}  // namespace tier4_deprecated_api_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_deprecated_api_adapter::ManualStatus)
