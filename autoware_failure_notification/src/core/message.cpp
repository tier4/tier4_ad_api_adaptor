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

#include "message.hpp"

#include <string>

namespace autoware::failure_notification
{

Message::Message(const Notification * parent, const YAML::Node yaml)
{
  parent_ = parent;
  text_ = yaml["test"].as<std::string>();
}

void Messages::load(const Notification * parent, const YAML::Node yaml)
{
  for (const auto & node : yaml) {
    entities_.emplace_back(std::make_unique<Message>(parent, node));
  }

  for (const auto & entity : entities_) {
    pointers_.push_back(entity.get());
  }
}

}  // namespace autoware::failure_notification
