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

#include "notification.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace autoware::failure_notification
{

Notification::Notification(const std::string & path, YAML::Node yaml, const Settings & settings)
: path_(path)
{
  if (const auto node = yaml["error_code"]) {
    error_code_ = node.as<std::string>();
  } else {
    throw std::runtime_error("error_code field is required in " + path);
  }

  if (const auto node = yaml["priority"]) {
    priority_ = node.as<int>();
  } else {
    throw std::runtime_error("priority field is required in " + path);
  }

  if (const auto node = yaml["notification_level"]) {
    notification_level_ = node.as<int>();
  } else {
    throw std::runtime_error("notification_level field is required in " + path);
  }

  if (const auto node = yaml["messages"]) {
    messages_ = std::make_unique<Messages>(this, node, settings);
  } else {
    throw std::runtime_error("messages field is required in " + path);
  }
}

void Notification::update(const Context & context, DiagLevel level)
{
  current_level_ = level;
  current_message_ = nullptr;

  if (level == DiagStatus::OK) return;

  for (const auto & message : messages_->list()) {
    if (message->condition()->evaluate(context)) {
      current_message_ = message;
      return;
    }
  }
}

Notifications::Notifications(YAML::Node yaml, const Settings & settings)
{
  const auto notifications = yaml["notifications"];

  for (const auto & iter : notifications) {
    const auto path = iter.first.as<std::string>();
    const auto node = iter.second;
    entities_.emplace_back(std::make_unique<Notification>(path, node, settings));
  }

  for (const auto & entity : entities_) {
    pointers_.push_back(entity.get());
  }

  const auto compare = [](const Notification * lhs, const Notification * rhs) {
    return lhs->priority() > rhs->priority();
  };
  std::sort(pointers_.begin(), pointers_.end(), compare);
}

}  // namespace autoware::failure_notification
