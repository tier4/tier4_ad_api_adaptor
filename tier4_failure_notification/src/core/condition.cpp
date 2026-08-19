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

#include "condition.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::failure_notification
{

std::unique_ptr<Condition> parse_expr(const Expression & expr)
{
  if (expr.data == "Always") {
    return std::make_unique<TrueCondition>();
  }
  if (expr.data == "True") {
    return std::make_unique<TrueCondition>();
  }
  if (expr.data == "False") {
    return std::make_unique<FalseCondition>();
  }
  if (expr.data == "Not") {
    return std::make_unique<NotCondition>(expr);
  }
  if (expr.data == "And") {
    return std::make_unique<AndCondition>(expr);
  }
  if (expr.data == "LocalizationState") {
    return std::make_unique<LocalizationStateCondition>(expr);
  }
  if (expr.data == "RouteState") {
    return std::make_unique<RouteStateCondition>(expr);
  }
  throw std::runtime_error("unknown condition: " + expr.data);
}

std::unique_ptr<Condition> Condition::parse(const std::string & str)
{
  return parse_expr(Expression::parse(str));
}

NotCondition::NotCondition(const Expression & expr)
{
  if (!expr.args || expr.args->size() != 1) {
    throw std::runtime_error("Not condition requires exactly one argument");
  }
  condition_ = parse_expr(expr.args->front());
}

bool NotCondition::evaluate(const Context & context) const
{
  return !condition_->evaluate(context);
}

AndCondition::AndCondition(const Expression & expr)
{
  if (!expr.args || expr.args->empty()) {
    throw std::runtime_error("And condition requires at least one argument");
  }
  for (const auto & arg : *expr.args) {
    conditions_.push_back(parse_expr(arg));
  }
}

bool AndCondition::evaluate(const Context & context) const
{
  for (const auto & condition : conditions_) {
    if (!condition->evaluate(context)) return false;
  }
  return true;
}

LocalizationStateCondition::LocalizationStateCondition(const Expression & expr)
{
  const auto get_state = [](const std::string & str) {
    if (str == "Unknown") return LocalizationState::UNKNOWN;
    if (str == "Uninitialized") return LocalizationState::UNINITIALIZED;
    if (str == "Initializing") return LocalizationState::INITIALIZING;
    if (str == "Initialized") return LocalizationState::INITIALIZED;
    throw std::runtime_error("Invalid LocalizationState: " + str);
  };

  if (!expr.args) {
    throw std::runtime_error("LocalizationState condition requires arguments");
  }
  for (const auto & arg : *expr.args) {
    states_.insert(get_state(arg.data));
  }
}

bool LocalizationStateCondition::evaluate(const Context & context) const
{
  return states_.count(context.localization_state.state) != 0;
}

RouteStateCondition::RouteStateCondition(const Expression & expr)
{
  const auto get_state = [](const std::string & str) {
    if (str == "Unknown") return RouteState::UNKNOWN;
    if (str == "Unset") return RouteState::UNSET;
    if (str == "Set") return RouteState::SET;
    if (str == "Arrived") return RouteState::ARRIVED;
    throw std::runtime_error("Invalid RouteState: " + str);
  };

  if (!expr.args) {
    throw std::runtime_error("RouteState condition requires arguments");
  }
  for (const auto & arg : *expr.args) {
    states_.insert(get_state(arg.data));
  }
}

bool RouteStateCondition::evaluate(const Context & context) const
{
  return states_.count(context.route_state.state) != 0;
}

}  // namespace autoware::failure_notification
