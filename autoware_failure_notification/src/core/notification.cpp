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
#include <string>

namespace autoware::failure_notification
{

Notification::Notification(const std::string & path, YAML::Node yaml) : path_(path)
{
  priority_ = yaml["priority"].as<int>(0);  // TODO(Takagi, Isamu): Remove default value.
  messages_.load(this, yaml["messages"]);
}

Notifications::Notifications(const std::string & path)
{
  const auto file = YAML::LoadFile(path);
  const auto root = file["notifications"];

  for (const auto & node : root) {
    const auto path = node.first.as<std::string>();
    const auto yaml = node.second;
    entities_.emplace_back(std::make_unique<Notification>(path, yaml));
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
