// Copyright 2026 The Autoware Contributors
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

#include "core/condition.hpp"
#include "core/context.hpp"

#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace autoware::failure_notification;  // NOLINT(build/namespaces)

Context default_context()
{
  Context context;
  context.route_state.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  context.route_state.state = Context::RouteState::UNKNOWN;
  context.localization_state.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  context.localization_state.state = Context::LocalizationState::UNKNOWN;
  return context;
}

bool evaluate(const std::string & expr, const Context & context = default_context())
{
  return Condition::parse(expr)->evaluate(context);
}

TEST(Condition, Always)
{
  EXPECT_TRUE(evaluate("Always"));
}

TEST(Condition, True)
{
  EXPECT_TRUE(evaluate("True"));
}

TEST(Condition, False)
{
  EXPECT_FALSE(evaluate("False"));
}

TEST(Condition, Not)
{
  EXPECT_FALSE(evaluate("Not(True)"));
  EXPECT_TRUE(evaluate("Not(False)"));
}

TEST(Condition, And2)
{
  // clang-format off
  EXPECT_FALSE(evaluate("And(False, False)"));
  EXPECT_FALSE(evaluate("And(True , False)"));
  EXPECT_FALSE(evaluate("And(False, True )"));
  EXPECT_TRUE (evaluate("And(True , True )"));
  // clang-format on
}

TEST(Condition, And3)
{
  // clang-format off
  EXPECT_FALSE(evaluate("And(False, False, False)"));
  EXPECT_FALSE(evaluate("And(True , False, False)"));
  EXPECT_FALSE(evaluate("And(False, True , False)"));
  EXPECT_FALSE(evaluate("And(True , True , False)"));
  EXPECT_FALSE(evaluate("And(False, False, True )"));
  EXPECT_FALSE(evaluate("And(True , False, True )"));
  EXPECT_FALSE(evaluate("And(False, True , True )"));
  EXPECT_TRUE (evaluate("And(True , True , True )"));
  // clang-format on
}
