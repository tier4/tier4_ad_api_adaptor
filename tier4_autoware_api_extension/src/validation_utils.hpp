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

#ifndef VALIDATION_UTILS_HPP_
#define VALIDATION_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <tier4_external_api_msgs/msg/response_status.hpp>
#include <tier4_external_api_msgs/msg/traffic_light_element.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace tier4_autoware_api_extension
{
namespace validation
{

// =============================================================================
// Core Validation Functions
// =============================================================================

/// Check if a floating point value is finite (not NaN or Inf)
template <typename T>
inline bool is_finite(const T value)
{
  return std::isfinite(value);
}

/// Check if a value is non-negative
template <typename T>
inline bool is_non_negative(const T value)
{
  return value >= static_cast<T>(0);
}

/// Check if a value is within a range [min_val, max_val]
template <typename T>
inline bool is_in_range(const T value, const T min_val, const T max_val)
{
  return value >= min_val && value <= max_val;
}

/// Check if a value is normalized [0.0, 1.0]
template <typename T>
inline bool is_normalized(const T value)
{
  return is_in_range(value, static_cast<T>(0.0), static_cast<T>(1.0));
}

// =============================================================================
// ValidationResult for Services
// =============================================================================

struct ValidationResult
{
  bool is_valid{true};
  std::string error_message;

  static ValidationResult success() { return ValidationResult{true, ""}; }

  static ValidationResult error(const std::string & message)
  {
    return ValidationResult{false, message};
  }

  ValidationResult & operator+=(const ValidationResult & other)
  {
    if (!other.is_valid) {
      is_valid = false;
      if (!error_message.empty()) {
        error_message += "; ";
      }
      error_message += other.error_message;
    }
    return *this;
  }
};

/// Set ResponseStatus based on ValidationResult
inline void set_response_status(
  tier4_external_api_msgs::msg::ResponseStatus & status, const ValidationResult & result)
{
  if (result.is_valid) {
    status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
    status.message = "";
  } else {
    status.code = tier4_external_api_msgs::msg::ResponseStatus::ERROR;
    status.message = result.error_message;
  }
}

// =============================================================================
// Clamping with Warning for Topics
// =============================================================================

/// Clamp a value to a range and log a warning if clamping occurred
template <typename T>
inline T clamp_with_warning(
  const T value, const T min_val, const T max_val, const std::string & field_name,
  const rclcpp::Logger & logger)
{
  if (!is_finite(value)) {
    RCLCPP_WARN(
      logger, "Invalid %s: value is not finite, clamping to %f", field_name.c_str(),
      static_cast<double>(min_val));
    return min_val;
  }
  if (value < min_val) {
    RCLCPP_WARN(
      logger, "Invalid %s: %f < %f, clamping to minimum", field_name.c_str(),
      static_cast<double>(value), static_cast<double>(min_val));
    return min_val;
  }
  if (value > max_val) {
    RCLCPP_WARN(
      logger, "Invalid %s: %f > %f, clamping to maximum", field_name.c_str(),
      static_cast<double>(value), static_cast<double>(max_val));
    return max_val;
  }
  return value;
}

/// Clamp to non-negative with warning
template <typename T>
inline T clamp_non_negative(
  const T value, const std::string & field_name, const rclcpp::Logger & logger)
{
  return clamp_with_warning(
    value, static_cast<T>(0), std::numeric_limits<T>::max(), field_name, logger);
}

/// Clamp to normalized range [0.0, 1.0] with warning
template <typename T>
inline T clamp_normalized(
  const T value, const std::string & field_name, const rclcpp::Logger & logger)
{
  return clamp_with_warning(value, static_cast<T>(0.0), static_cast<T>(1.0), field_name, logger);
}

// =============================================================================
// Field-Specific Validators
// =============================================================================

/// Validate velocity (must be finite and non-negative)
inline ValidationResult validate_velocity(const float velocity)
{
  if (!is_finite(velocity)) {
    return ValidationResult::error("velocity is not finite (NaN or Inf)");
  }
  if (!is_non_negative(velocity)) {
    return ValidationResult::error("velocity must be non-negative");
  }
  return ValidationResult::success();
}

/// Validate distance (must be finite and non-negative)
inline ValidationResult validate_distance(const double distance)
{
  if (!is_finite(distance)) {
    return ValidationResult::error("distance is not finite (NaN or Inf)");
  }
  if (!is_non_negative(distance)) {
    return ValidationResult::error("distance must be non-negative");
  }
  return ValidationResult::success();
}

/// Validate confidence (must be in [0.0, 1.0])
inline ValidationResult validate_confidence(const float confidence)
{
  if (!is_finite(confidence)) {
    return ValidationResult::error("confidence is not finite (NaN or Inf)");
  }
  if (!is_normalized(confidence)) {
    return ValidationResult::error("confidence must be in range [0.0, 1.0]");
  }
  return ValidationResult::success();
}

// =============================================================================
// Enum Validators
// =============================================================================

/// Generic enum validator using constexpr array of valid values
template <typename T, std::size_t N>
constexpr bool is_valid_enum(const T value, const std::array<T, N> & valid_values)
{
  for (const auto & v : valid_values) {
    if (value == v) return true;
  }
  return false;
}

// Message type aliases for readability
using TrafficLightElement = tier4_external_api_msgs::msg::TrafficLightElement;
using PlanningFactor = autoware_internal_planning_msgs::msg::PlanningFactor;

/// TrafficLightElement color: UNKNOWN, RED, AMBER, GREEN, WHITE
inline constexpr std::array<uint8_t, 5> TRAFFIC_LIGHT_COLORS = {
  TrafficLightElement::UNKNOWN, TrafficLightElement::RED, TrafficLightElement::AMBER,
  TrafficLightElement::GREEN, TrafficLightElement::WHITE};

/// TrafficLightElement shape: UNKNOWN, CIRCLE, LEFT_ARROW, RIGHT_ARROW, UP_ARROW,
/// UP_LEFT_ARROW, UP_RIGHT_ARROW, DOWN_ARROW, DOWN_LEFT_ARROW, DOWN_RIGHT_ARROW, CROSS
inline constexpr std::array<uint8_t, 11> TRAFFIC_LIGHT_SHAPES = {
  TrafficLightElement::UNKNOWN,
  TrafficLightElement::CIRCLE,
  TrafficLightElement::LEFT_ARROW,
  TrafficLightElement::RIGHT_ARROW,
  TrafficLightElement::UP_ARROW,
  TrafficLightElement::UP_LEFT_ARROW,
  TrafficLightElement::UP_RIGHT_ARROW,
  TrafficLightElement::DOWN_ARROW,
  TrafficLightElement::DOWN_LEFT_ARROW,
  TrafficLightElement::DOWN_RIGHT_ARROW,
  TrafficLightElement::CROSS};

/// TrafficLightElement status: UNKNOWN, SOLID_OFF, SOLID_ON, FLASHING
inline constexpr std::array<uint8_t, 4> TRAFFIC_LIGHT_STATUSES = {
  TrafficLightElement::UNKNOWN, TrafficLightElement::SOLID_OFF, TrafficLightElement::SOLID_ON,
  TrafficLightElement::FLASHING};

/// PlanningFactor behavior: UNKNOWN, NONE, SLOW_DOWN, STOP, SHIFT_LEFT, SHIFT_RIGHT,
/// TURN_LEFT, TURN_RIGHT
inline constexpr std::array<uint16_t, 8> BEHAVIOR_TYPES = {
  PlanningFactor::UNKNOWN,   PlanningFactor::NONE,       PlanningFactor::SLOW_DOWN,
  PlanningFactor::STOP,      PlanningFactor::SHIFT_LEFT, PlanningFactor::SHIFT_RIGHT,
  PlanningFactor::TURN_LEFT, PlanningFactor::TURN_RIGHT};

/// Validate TrafficLightElement color enum
inline bool is_valid_traffic_light_color(const uint8_t color)
{
  return is_valid_enum(color, TRAFFIC_LIGHT_COLORS);
}

/// Validate TrafficLightElement shape enum
inline bool is_valid_traffic_light_shape(const uint8_t shape)
{
  return is_valid_enum(shape, TRAFFIC_LIGHT_SHAPES);
}

/// Validate TrafficLightElement status enum
inline bool is_valid_traffic_light_status(const uint8_t status)
{
  return is_valid_enum(status, TRAFFIC_LIGHT_STATUSES);
}

/// Validate PlanningFactor behavior_type enum
inline bool is_valid_behavior_type(const uint16_t behavior_type)
{
  return is_valid_enum(behavior_type, BEHAVIOR_TYPES);
}

}  // namespace validation
}  // namespace tier4_autoware_api_extension

#endif  // VALIDATION_UTILS_HPP_
