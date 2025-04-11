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

#include "manual_control.hpp"

#include <string>
#include <utility>

namespace tier4_deprecated_api_adapter
{

ManualControl::ManualControl(const rclcpp::NodeOptions & options) : Node("manual_control", options)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  const auto mode = declare_parameter<std::string>("mode", descriptor);
  const auto bind = [this](auto && func) { return std::bind(func, this, std::placeholders::_1); };

  {
    const auto ns = "/api/external/set/command/" + mode + "/";
    sub_heartbeat_ = create_subscription<ExternalHeartbeat>(
      ns + "heartbeat", 1, bind(&ManualControl::relay_heartbeat));
    sub_control_ =
      create_subscription<ExternalControl>(ns + "control", 1, bind(&ManualControl::relay_control));
    sub_gear_ =
      create_subscription<ExternalGear>(ns + "shift", 1, bind(&ManualControl::relay_gear));
    sub_turn_signal_ = create_subscription<ExternalTurnSignal>(
      ns + "turn_signal", 1, bind(&ManualControl::relay_turn_signal));
  }

  {
    const auto ns = "/external/" + mode + "/";
    pub_heartbeat_ = create_publisher<InternalHeartbeat>(ns + "heartbeat", 1);
    pub_pedals_ = create_publisher<InternalPedals>(ns + "pedals_cmd", 1);
    pub_steering_ = create_publisher<InternalSteering>(ns + "steering_cmd", 1);
    pub_gear_ = create_publisher<InternalGear>(ns + "gear_cmd", 1);
    pub_turn_indicators_ = create_publisher<InternalTurnIndicators>(ns + "turn_indicators_cmd", 1);
    pub_hazard_lights_ = create_publisher<InternalHazardLights>(ns + "hazard_lights_cmd", 1);
  }
}

void ManualControl::relay_heartbeat(const ExternalHeartbeat & msg)
{
  InternalHeartbeat heartbeat;
  heartbeat.stamp = msg.stamp;
  heartbeat.ready = true;
  pub_heartbeat_->publish(heartbeat);
}

void ManualControl::relay_control(const ExternalControl & msg)
{
  InternalPedals pedals;
  pedals.stamp = msg.stamp;
  pedals.throttle = msg.control.throttle;
  pedals.brake = msg.control.brake;
  pub_pedals_->publish(pedals);

  InternalSteering steering;
  steering.stamp = msg.stamp;
  steering.steering_tire_angle = msg.control.steering_angle;
  pub_steering_->publish(steering);
}

void ManualControl::relay_gear(const ExternalGear & msg)
{
  // clang-format off
  const auto convert = [](const uint8_t gear)
  {
    using ExternalType = ExternalGear::_gear_shift_type;
    switch(gear) {
      case ExternalType::NONE:    return InternalGear::NONE;
      case ExternalType::PARKING: return InternalGear::PARK;
      case ExternalType::REVERSE: return InternalGear::REVERSE;
      case ExternalType::NEUTRAL: return InternalGear::NEUTRAL;
      case ExternalType::DRIVE:   return InternalGear::DRIVE;
      case ExternalType::LOW:     return InternalGear::LOW;
      default:                    return InternalGear::NONE;
    }
  };
  // clang-format on

  InternalGear gear;
  gear.stamp = msg.stamp;
  gear.command = convert(msg.gear_shift.data);
  pub_gear_->publish(gear);
}

void ManualControl::relay_turn_signal(const ExternalTurnSignal & msg)
{
  // clang-format off
  const auto convert = [](const uint8_t turn_signal) -> std::pair<uint8_t, uint8_t>
  {
    using ExternalType = ExternalTurnSignal::_turn_signal_type;
    switch(turn_signal) {
      case ExternalType::NONE:    return {InternalTurnIndicators::DISABLE,      InternalHazardLights::DISABLE};     // NOLINT
      case ExternalType::LEFT:    return {InternalTurnIndicators::ENABLE_LEFT,  InternalHazardLights::DISABLE};     // NOLINT
      case ExternalType::RIGHT:   return {InternalTurnIndicators::ENABLE_RIGHT, InternalHazardLights::DISABLE};     // NOLINT
      case ExternalType::HAZARD:  return {InternalTurnIndicators::DISABLE,      InternalHazardLights::ENABLE};      // NOLINT
      default:                    return {InternalTurnIndicators::NO_COMMAND,   InternalHazardLights::NO_COMMAND};  // NOLINT
    }
  };
  // clang-format on

  const auto [turn_indicators_data, hazard_lights_data] = convert(msg.turn_signal.data);

  InternalTurnIndicators turn_indicators;
  turn_indicators.stamp = msg.stamp;
  turn_indicators.command = turn_indicators_data;
  pub_turn_indicators_->publish(turn_indicators);

  InternalHazardLights hazard_lights;
  hazard_lights.stamp = msg.stamp;
  hazard_lights.command = hazard_lights_data;
  pub_hazard_lights_->publish(hazard_lights);
}

}  // namespace tier4_deprecated_api_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_deprecated_api_adapter::ManualControl)
