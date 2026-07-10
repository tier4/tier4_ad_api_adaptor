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

#ifndef CORE__CONDITION_HPP_
#define CORE__CONDITION_HPP_

#include "context.hpp"
#include "expression.hpp"

#include <memory>
#include <string>
#include <unordered_set>

namespace autoware::failure_notification
{

class Condition
{
public:
  static std::unique_ptr<Condition> parse(const std::string & str);
  virtual ~Condition() = default;
  virtual bool evaluate(const Context & context) const = 0;
};

class TrueCondition : public Condition
{
public:
  bool evaluate(const Context & context) const override;
};

class NotCondition : public Condition
{
public:
  explicit NotCondition(const Expression & expr);
  bool evaluate(const Context & context) const override;

private:
  std::unique_ptr<Condition> condition_;
};

class LocalizationStateCondition : public Condition
{
public:
  explicit LocalizationStateCondition(const Expression & expr);
  bool evaluate(const Context & context) const override;

private:
  using LocalizationState = Context::LocalizationState;
  std::unordered_set<LocalizationState::_state_type> states_;
};

class RouteStateCondition : public Condition
{
public:
  explicit RouteStateCondition(const Expression & expr);
  bool evaluate(const Context & context) const override;

private:
  using RouteState = Context::RouteState;
  std::unordered_set<RouteState::_state_type> states_;
};

}  // namespace autoware::failure_notification

#endif  // CORE__CONDITION_HPP_
