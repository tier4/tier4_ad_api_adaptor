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

#include "failure.hpp"

#include "notification.hpp"

#include <string>
#include <vector>

namespace autoware::failure_notification
{

Failure::Failure(const Notification * parent, const YAML::Node yaml) : parent_(parent)
{
  if (const auto node = yaml["condition"]) {
    condition_ = Condition::parse(node.as<std::string>());
  }

  if (const auto node = yaml["code"]) {
    code_ = node.as<std::string>();
  } else {
    throw std::runtime_error("code field is required in " + parent->path());
  }
}

Failures::Failures(const Notification * parent, const YAML::Node yaml)
{
  for (const auto & node : yaml) {
    entities_.emplace_back(std::make_unique<Failure>(parent, node));
  }

  for (const auto & entity : entities_) {
    pointers_.push_back(entity.get());
  }
}

}  // namespace autoware::failure_notification
