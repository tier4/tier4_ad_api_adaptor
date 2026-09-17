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

#include <string>
#include <vector>

using namespace autoware::failure_notification;  // NOLINT(build/namespaces)

namespace autoware::failure_notification
{
bool operator==(const Expression & x, const Expression & y)
{
  return (x.data == y.data) && (x.args == y.args);
}
}  // namespace autoware::failure_notification

Expression v(const std::string & name)
{
  return Expression{name, std::nullopt};
}

template <typename... Args>
Expression f(const std::string & name, Args... args)
{
  return Expression{name, std::vector<Expression>{args...}};
}

TEST(Expression, Constant)
{
  const auto expr = Expression::parse("constant");
  EXPECT_EQ(expr, v("constant"));
}

TEST(Expression, FunctionNoArgs)
{
  const auto expr = Expression::parse("function()");
  EXPECT_EQ(expr, f("function"));
}

TEST(Expression, FunctionOneArg)
{
  const auto expr = Expression::parse("function(one)");
  EXPECT_EQ(expr, f("function", v("one")));
}

TEST(Expression, FunctionTwoArgs)
{
  const auto expr = Expression::parse("function(one, two)");
  EXPECT_EQ(expr, f("function", v("one"), v("two")));
}

TEST(Expression, FunctionNested)
{
  const auto expr = Expression::parse("function(A, B(C), D(E(F), G))");
  EXPECT_EQ(expr, f("function", v("A"), f("B", v("C")), f("D", f("E", v("F")), v("G"))));
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
