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

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::failure_notification
{

Notification::Notification(const std::string & path, YAML::Node yaml) : path_(path)
{
  if (const auto node = yaml["failures"]) {
    failures_ = std::make_unique<Failures>(this, node);
  } else {
    throw std::runtime_error("failures field is required in " + path);
  }
}

void Notification::update(const Context & context, DiagLevel level)
{
  current_failure_ = nullptr;
  if (level == DiagStatus::OK) return;

  for (const auto & failure : failures_->list()) {
    if (failure->condition()->evaluate(context)) {
      current_failure_ = failure;
      return;
    }
  }
}

Notifications::Notifications(YAML::Node yaml)
{
  const auto notifications = yaml["notifications"];

  for (const auto & iter : notifications) {
    const auto path = iter.first.as<std::string>();
    const auto node = iter.second;
    entities_.emplace_back(std::make_unique<Notification>(path, node));
  }

  for (const auto & entity : entities_) {
    pointers_.push_back(entity.get());
  }
}

}  // namespace autoware::failure_notification
