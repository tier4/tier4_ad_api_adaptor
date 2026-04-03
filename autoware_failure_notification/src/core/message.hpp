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

#ifndef CORE__MESSAGE_HPP_
#define CORE__MESSAGE_HPP_

#include "condition.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::failure_notification
{

class Notification;

class Message
{
public:
  Message(const Notification * parent, const YAML::Node yaml);
  const auto & text() const { return text_; }
  const auto & condition() const { return condition_; }

private:
  const Notification * parent_;
  std::string text_;
  std::unique_ptr<Condition> condition_;
};

class Messages
{
public:
  Messages(const Notification * parent, const YAML::Node yaml);
  const auto & messages() const { return pointers_; }

private:
  std::vector<std::unique_ptr<Message>> entities_;
  std::vector<Message *> pointers_;
};

}  // namespace autoware::failure_notification

#endif  // CORE__MESSAGE_HPP_
