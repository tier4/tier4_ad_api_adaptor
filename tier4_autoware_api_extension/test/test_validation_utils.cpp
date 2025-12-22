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

#include "../src/validation_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/response_status.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

using tier4_autoware_api_extension::validation::BEHAVIOR_TYPES;
using tier4_autoware_api_extension::validation::clamp_non_negative;
using tier4_autoware_api_extension::validation::clamp_normalized;
using tier4_autoware_api_extension::validation::clamp_with_warning;
using tier4_autoware_api_extension::validation::is_finite;
using tier4_autoware_api_extension::validation::is_in_range;
using tier4_autoware_api_extension::validation::is_non_negative;
using tier4_autoware_api_extension::validation::is_normalized;
using tier4_autoware_api_extension::validation::is_valid_behavior_type;
using tier4_autoware_api_extension::validation::is_valid_enum;
using tier4_autoware_api_extension::validation::is_valid_traffic_light_color;
using tier4_autoware_api_extension::validation::is_valid_traffic_light_shape;
using tier4_autoware_api_extension::validation::is_valid_traffic_light_status;
using tier4_autoware_api_extension::validation::set_response_status;
using tier4_autoware_api_extension::validation::TRAFFIC_LIGHT_COLORS;
using tier4_autoware_api_extension::validation::TRAFFIC_LIGHT_SHAPES;
using tier4_autoware_api_extension::validation::TRAFFIC_LIGHT_STATUSES;
using tier4_autoware_api_extension::validation::validate_confidence;
using tier4_autoware_api_extension::validation::validate_distance;
using tier4_autoware_api_extension::validation::validate_velocity;
using tier4_autoware_api_extension::validation::ValidationResult;

// =============================================================================
// is_finite tests
// =============================================================================

TEST(ValidationUtilsTest, IsFinite_NormalValues)
{
  EXPECT_TRUE(is_finite(0.0));
  EXPECT_TRUE(is_finite(1.0));
  EXPECT_TRUE(is_finite(-1.0));
  EXPECT_TRUE(is_finite(1e10));
  EXPECT_TRUE(is_finite(-1e10));
  EXPECT_TRUE(is_finite(0.0f));
  EXPECT_TRUE(is_finite(1.0f));
}

TEST(ValidationUtilsTest, IsFinite_NaN)
{
  EXPECT_FALSE(is_finite(std::nan("")));
  EXPECT_FALSE(is_finite(std::nanf("")));
  EXPECT_FALSE(is_finite(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(is_finite(std::numeric_limits<float>::quiet_NaN()));
}

TEST(ValidationUtilsTest, IsFinite_Infinity)
{
  EXPECT_FALSE(is_finite(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(is_finite(-std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(is_finite(std::numeric_limits<float>::infinity()));
  EXPECT_FALSE(is_finite(-std::numeric_limits<float>::infinity()));
}

// =============================================================================
// is_non_negative tests
// =============================================================================

TEST(ValidationUtilsTest, IsNonNegative_Positive)
{
  EXPECT_TRUE(is_non_negative(0.0));
  EXPECT_TRUE(is_non_negative(0.0f));
  EXPECT_TRUE(is_non_negative(1.0));
  EXPECT_TRUE(is_non_negative(1e10));
}

TEST(ValidationUtilsTest, IsNonNegative_Negative)
{
  EXPECT_FALSE(is_non_negative(-0.001));
  EXPECT_FALSE(is_non_negative(-1.0));
  EXPECT_FALSE(is_non_negative(-1e10));
}

// =============================================================================
// is_in_range tests
// =============================================================================

TEST(ValidationUtilsTest, IsInRange_InRange)
{
  EXPECT_TRUE(is_in_range(0.5, 0.0, 1.0));
  EXPECT_TRUE(is_in_range(0.0, 0.0, 1.0));
  EXPECT_TRUE(is_in_range(1.0, 0.0, 1.0));
  EXPECT_TRUE(is_in_range(5, 0, 10));
}

TEST(ValidationUtilsTest, IsInRange_OutOfRange)
{
  EXPECT_FALSE(is_in_range(-0.1, 0.0, 1.0));
  EXPECT_FALSE(is_in_range(1.1, 0.0, 1.0));
  EXPECT_FALSE(is_in_range(-1, 0, 10));
  EXPECT_FALSE(is_in_range(11, 0, 10));
}

// =============================================================================
// is_normalized tests
// =============================================================================

TEST(ValidationUtilsTest, IsNormalized_Valid)
{
  EXPECT_TRUE(is_normalized(0.0));
  EXPECT_TRUE(is_normalized(0.5));
  EXPECT_TRUE(is_normalized(1.0));
  EXPECT_TRUE(is_normalized(0.0f));
  EXPECT_TRUE(is_normalized(0.5f));
  EXPECT_TRUE(is_normalized(1.0f));
}

TEST(ValidationUtilsTest, IsNormalized_Invalid)
{
  EXPECT_FALSE(is_normalized(-0.1));
  EXPECT_FALSE(is_normalized(1.1));
  EXPECT_FALSE(is_normalized(-0.1f));
  EXPECT_FALSE(is_normalized(1.1f));
}

// =============================================================================
// ValidationResult tests
// =============================================================================

TEST(ValidationUtilsTest, ValidationResult_Success)
{
  auto result = ValidationResult::success();
  EXPECT_TRUE(result.is_valid);
  EXPECT_TRUE(result.error_message.empty());
}

TEST(ValidationUtilsTest, ValidationResult_Error)
{
  auto result = ValidationResult::error("test error");
  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.error_message, "test error");
}

TEST(ValidationUtilsTest, ValidationResult_Combine)
{
  auto result1 = ValidationResult::success();
  auto result2 = ValidationResult::error("error1");
  auto result3 = ValidationResult::error("error2");

  result1 += result2;
  EXPECT_FALSE(result1.is_valid);
  EXPECT_EQ(result1.error_message, "error1");

  result1 += result3;
  EXPECT_FALSE(result1.is_valid);
  EXPECT_EQ(result1.error_message, "error1; error2");
}

// =============================================================================
// set_response_status tests
// =============================================================================

TEST(ValidationUtilsTest, SetResponseStatus_Success)
{
  tier4_external_api_msgs::msg::ResponseStatus status;
  auto result = ValidationResult::success();
  set_response_status(status, result);

  EXPECT_EQ(status.code, tier4_external_api_msgs::msg::ResponseStatus::SUCCESS);
  EXPECT_TRUE(status.message.empty());
}

TEST(ValidationUtilsTest, SetResponseStatus_Error)
{
  tier4_external_api_msgs::msg::ResponseStatus status;
  auto result = ValidationResult::error("validation failed");
  set_response_status(status, result);

  EXPECT_EQ(status.code, tier4_external_api_msgs::msg::ResponseStatus::ERROR);
  EXPECT_EQ(status.message, "validation failed");
}

// =============================================================================
// clamp_with_warning tests
// =============================================================================

class ClampWithWarningTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(ClampWithWarningTest, InRange)
{
  auto result = clamp_with_warning(0.5, 0.0, 1.0, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 0.5);
}

TEST_F(ClampWithWarningTest, BelowMin)
{
  auto result = clamp_with_warning(-0.5, 0.0, 1.0, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST_F(ClampWithWarningTest, AboveMax)
{
  auto result = clamp_with_warning(1.5, 0.0, 1.0, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 1.0);
}

TEST_F(ClampWithWarningTest, NaN)
{
  auto result = clamp_with_warning(std::nan(""), 0.0, 1.0, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST_F(ClampWithWarningTest, Infinity)
{
  auto result = clamp_with_warning(
    std::numeric_limits<double>::infinity(), 0.0, 1.0, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST_F(ClampWithWarningTest, ClampNonNegative_Valid)
{
  auto result = clamp_non_negative(5.0, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 5.0);
}

TEST_F(ClampWithWarningTest, ClampNonNegative_Negative)
{
  auto result = clamp_non_negative(-5.0, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST_F(ClampWithWarningTest, ClampNormalized_Valid)
{
  auto result = clamp_normalized(0.5, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 0.5);
}

TEST_F(ClampWithWarningTest, ClampNormalized_Above)
{
  auto result = clamp_normalized(1.5, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 1.0);
}

TEST_F(ClampWithWarningTest, ClampNormalized_Below)
{
  auto result = clamp_normalized(-0.5, "test_field", node_->get_logger());
  EXPECT_DOUBLE_EQ(result, 0.0);
}

// =============================================================================
// validate_velocity tests
// =============================================================================

TEST(ValidationUtilsTest, ValidateVelocity_Valid)
{
  auto result = validate_velocity(10.0f);
  EXPECT_TRUE(result.is_valid);
}

TEST(ValidationUtilsTest, ValidateVelocity_Zero)
{
  auto result = validate_velocity(0.0f);
  EXPECT_TRUE(result.is_valid);
}

TEST(ValidationUtilsTest, ValidateVelocity_Negative)
{
  auto result = validate_velocity(-1.0f);
  EXPECT_FALSE(result.is_valid);
  EXPECT_FALSE(result.error_message.empty());
}

TEST(ValidationUtilsTest, ValidateVelocity_NaN)
{
  auto result = validate_velocity(std::nanf(""));
  EXPECT_FALSE(result.is_valid);
  EXPECT_FALSE(result.error_message.empty());
}

TEST(ValidationUtilsTest, ValidateVelocity_Infinity)
{
  auto result = validate_velocity(std::numeric_limits<float>::infinity());
  EXPECT_FALSE(result.is_valid);
  EXPECT_FALSE(result.error_message.empty());
}

// =============================================================================
// validate_distance tests
// =============================================================================

TEST(ValidationUtilsTest, ValidateDistance_Valid)
{
  auto result = validate_distance(100.0);
  EXPECT_TRUE(result.is_valid);
}

TEST(ValidationUtilsTest, ValidateDistance_Zero)
{
  auto result = validate_distance(0.0);
  EXPECT_TRUE(result.is_valid);
}

TEST(ValidationUtilsTest, ValidateDistance_Negative)
{
  auto result = validate_distance(-1.0);
  EXPECT_FALSE(result.is_valid);
}

TEST(ValidationUtilsTest, ValidateDistance_NaN)
{
  auto result = validate_distance(std::nan(""));
  EXPECT_FALSE(result.is_valid);
}

// =============================================================================
// validate_confidence tests
// =============================================================================

TEST(ValidationUtilsTest, ValidateConfidence_Valid)
{
  EXPECT_TRUE(validate_confidence(0.0f).is_valid);
  EXPECT_TRUE(validate_confidence(0.5f).is_valid);
  EXPECT_TRUE(validate_confidence(1.0f).is_valid);
}

TEST(ValidationUtilsTest, ValidateConfidence_OutOfRange)
{
  EXPECT_FALSE(validate_confidence(-0.1f).is_valid);
  EXPECT_FALSE(validate_confidence(1.1f).is_valid);
}

TEST(ValidationUtilsTest, ValidateConfidence_NaN)
{
  EXPECT_FALSE(validate_confidence(std::nanf("")).is_valid);
}

// =============================================================================
// is_valid_enum template tests
// =============================================================================

TEST(ValidationUtilsTest, IsValidEnum_CustomArray)
{
  constexpr std::array<int, 3> valid_values = {1, 5, 10};
  EXPECT_TRUE(is_valid_enum(1, valid_values));
  EXPECT_TRUE(is_valid_enum(5, valid_values));
  EXPECT_TRUE(is_valid_enum(10, valid_values));
  EXPECT_FALSE(is_valid_enum(0, valid_values));
  EXPECT_FALSE(is_valid_enum(2, valid_values));
  EXPECT_FALSE(is_valid_enum(100, valid_values));
}

TEST(ValidationUtilsTest, IsValidEnum_EmptyArray)
{
  constexpr std::array<int, 0> empty_values = {};
  EXPECT_FALSE(is_valid_enum(0, empty_values));
  EXPECT_FALSE(is_valid_enum(1, empty_values));
}

TEST(ValidationUtilsTest, IsValidEnum_WithPredefinedArrays)
{
  // Test using predefined constexpr arrays
  EXPECT_TRUE(is_valid_enum(static_cast<uint8_t>(0), TRAFFIC_LIGHT_COLORS));
  EXPECT_TRUE(is_valid_enum(static_cast<uint8_t>(4), TRAFFIC_LIGHT_COLORS));
  EXPECT_FALSE(is_valid_enum(static_cast<uint8_t>(5), TRAFFIC_LIGHT_COLORS));

  EXPECT_TRUE(is_valid_enum(static_cast<uint8_t>(0), TRAFFIC_LIGHT_SHAPES));
  EXPECT_TRUE(is_valid_enum(static_cast<uint8_t>(10), TRAFFIC_LIGHT_SHAPES));
  EXPECT_FALSE(is_valid_enum(static_cast<uint8_t>(11), TRAFFIC_LIGHT_SHAPES));

  EXPECT_TRUE(is_valid_enum(static_cast<uint8_t>(0), TRAFFIC_LIGHT_STATUSES));
  EXPECT_TRUE(is_valid_enum(static_cast<uint8_t>(3), TRAFFIC_LIGHT_STATUSES));
  EXPECT_FALSE(is_valid_enum(static_cast<uint8_t>(4), TRAFFIC_LIGHT_STATUSES));

  EXPECT_TRUE(is_valid_enum(static_cast<uint16_t>(0), BEHAVIOR_TYPES));
  EXPECT_TRUE(is_valid_enum(static_cast<uint16_t>(7), BEHAVIOR_TYPES));
  EXPECT_FALSE(is_valid_enum(static_cast<uint16_t>(8), BEHAVIOR_TYPES));
}

// =============================================================================
// Enum validator wrapper tests
// =============================================================================

TEST(ValidationUtilsTest, IsValidTrafficLightColor)
{
  EXPECT_TRUE(is_valid_traffic_light_color(0));   // UNKNOWN
  EXPECT_TRUE(is_valid_traffic_light_color(1));   // RED
  EXPECT_TRUE(is_valid_traffic_light_color(2));   // AMBER
  EXPECT_TRUE(is_valid_traffic_light_color(3));   // GREEN
  EXPECT_TRUE(is_valid_traffic_light_color(4));   // WHITE
  EXPECT_FALSE(is_valid_traffic_light_color(5));  // Invalid
  EXPECT_FALSE(is_valid_traffic_light_color(255));
}

TEST(ValidationUtilsTest, IsValidTrafficLightShape)
{
  EXPECT_TRUE(is_valid_traffic_light_shape(0));    // UNKNOWN
  EXPECT_TRUE(is_valid_traffic_light_shape(1));    // CIRCLE
  EXPECT_TRUE(is_valid_traffic_light_shape(10));   // CROSS
  EXPECT_FALSE(is_valid_traffic_light_shape(11));  // Invalid
  EXPECT_FALSE(is_valid_traffic_light_shape(255));
}

TEST(ValidationUtilsTest, IsValidTrafficLightStatus)
{
  EXPECT_TRUE(is_valid_traffic_light_status(0));   // UNKNOWN
  EXPECT_TRUE(is_valid_traffic_light_status(1));   // SOLID_OFF
  EXPECT_TRUE(is_valid_traffic_light_status(2));   // SOLID_ON
  EXPECT_TRUE(is_valid_traffic_light_status(3));   // FLASHING
  EXPECT_FALSE(is_valid_traffic_light_status(4));  // Invalid
  EXPECT_FALSE(is_valid_traffic_light_status(255));
}

TEST(ValidationUtilsTest, IsValidBehaviorType)
{
  EXPECT_TRUE(is_valid_behavior_type(0));   // UNKNOWN
  EXPECT_TRUE(is_valid_behavior_type(1));   // NONE
  EXPECT_TRUE(is_valid_behavior_type(7));   // TURN_RIGHT
  EXPECT_FALSE(is_valid_behavior_type(8));  // Invalid
  EXPECT_FALSE(is_valid_behavior_type(255));
}
