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

#ifndef MANUAL_CONTROL_HPP_
#define MANUAL_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/gear_shift_stamped.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>
#include <tier4_external_api_msgs/msg/turn_signal_stamped.hpp>

namespace tier4_deprecated_api_adapter
{

using ExternalHeartbeat = tier4_external_api_msgs::msg::Heartbeat;
using ExternalControl = tier4_external_api_msgs::msg::ControlCommandStamped;
using ExternalGear = tier4_external_api_msgs::msg::GearShiftStamped;
using ExternalTurnSignal = tier4_external_api_msgs::msg::TurnSignalStamped;

using InternalHeartbeat = autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
using InternalPedals = autoware_adapi_v1_msgs::msg::PedalsCommand;
using InternalSteering = autoware_adapi_v1_msgs::msg::SteeringCommand;
using InternalGear = autoware_vehicle_msgs::msg::GearCommand;
using InternalTurnIndicators = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using InternalHazardLights = autoware_vehicle_msgs::msg::HazardLightsCommand;

class ManualControl : public rclcpp::Node
{
public:
  explicit ManualControl(const rclcpp::NodeOptions & options);

private:
  // Command input.
  rclcpp::Subscription<ExternalHeartbeat>::SharedPtr sub_heartbeat_;
  rclcpp::Subscription<ExternalControl>::SharedPtr sub_control_;
  rclcpp::Subscription<ExternalGear>::SharedPtr sub_gear_;
  rclcpp::Subscription<ExternalTurnSignal>::SharedPtr sub_turn_signal_;

  // Command output.
  rclcpp::Publisher<InternalHeartbeat>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<InternalPedals>::SharedPtr pub_pedals_;
  rclcpp::Publisher<InternalSteering>::SharedPtr pub_steering_;
  rclcpp::Publisher<InternalGear>::SharedPtr pub_gear_;
  rclcpp::Publisher<InternalTurnIndicators>::SharedPtr pub_turn_indicators_;
  rclcpp::Publisher<InternalHazardLights>::SharedPtr pub_hazard_lights_;

  // Callbacks.
  void relay_heartbeat(const ExternalHeartbeat & msg);
  void relay_control(const ExternalControl & msg);
  void relay_gear(const ExternalGear & msg);
  void relay_turn_signal(const ExternalTurnSignal & msg);
};

}  // namespace tier4_deprecated_api_adapter

#endif  // MANUAL_CONTROL_HPP_
