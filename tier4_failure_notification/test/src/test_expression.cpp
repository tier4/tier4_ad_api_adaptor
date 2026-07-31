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

#include "core/expression.hpp"

#include <gtest/gtest.h>

using namespace autoware::failure_notification;  // NOLINT(build/namespaces)

TEST(Expression, Constant)
{
  const auto expr = Expression::parse("constant");
  EXPECT_EQ(expr.data, "constant");
  EXPECT_FALSE(expr.args.has_value());
}

TEST(Expression, FunctionNoArgs)
{
  const auto expr = Expression::parse("function()");
  EXPECT_EQ(expr.data, "function");
  EXPECT_TRUE(expr.args.has_value());
  EXPECT_EQ(expr.args->size(), 0);
}

TEST(Expression, FunctionOneArg)
{
  const auto expr = Expression::parse("function(one)");
  EXPECT_EQ(expr.data, "function");
  EXPECT_TRUE(expr.args.has_value());
  EXPECT_EQ(expr.args->size(), 1);
  EXPECT_EQ(expr.args->at(0).data, "one");
}

TEST(Expression, FunctionTwoArgs)
{
  const auto expr = Expression::parse("function(one, two)");
  EXPECT_EQ(expr.data, "function");
  EXPECT_TRUE(expr.args.has_value());
  EXPECT_EQ(expr.args->size(), 2);
  EXPECT_EQ(expr.args->at(0).data, "one");
  EXPECT_EQ(expr.args->at(1).data, "two");
}

TEST(Expression, FunctionNoClose)
{
  EXPECT_THROW(Expression::parse("function(one, two"), std::runtime_error);
}

TEST(Expression, FunctionOverClose)
{
  EXPECT_THROW(Expression::parse("function(one, two))"), std::runtime_error);
}

TEST(Expression, FunctionNoOpen)
{
  EXPECT_THROW(Expression::parse("function one, two)"), std::runtime_error);
}

TEST(Expression, FunctionOverOpen)
{
  EXPECT_THROW(Expression::parse("function((one, two)"), std::runtime_error);
}

TEST(Expression, FunctionNested)
{
  const auto expr = Expression::parse("function(one, two(), three(four, five))");
  EXPECT_EQ(expr.data, "function");
  EXPECT_TRUE(expr.args.has_value());
  EXPECT_EQ(expr.args->size(), 3);
  EXPECT_EQ(expr.args->at(0).data, "one");
  EXPECT_EQ(expr.args->at(1).data, "two");
  EXPECT_TRUE(expr.args->at(1).args.has_value());
  EXPECT_EQ(expr.args->at(1).args->size(), 0);
  EXPECT_EQ(expr.args->at(2).data, "three");
  EXPECT_TRUE(expr.args->at(2).args.has_value());
  EXPECT_EQ(expr.args->at(2).args->size(), 2);
  EXPECT_EQ(expr.args->at(2).args->at(0).data, "four");
  EXPECT_EQ(expr.args->at(2).args->at(1).data, "five");
}
