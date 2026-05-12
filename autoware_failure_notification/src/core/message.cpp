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

#include "notification.hpp"

#include <string>
#include <vector>

namespace autoware::failure_notification
{

Message::Message(const Notification * parent, const YAML::Node yaml, const Settings & settings)
: parent_(parent)
{
  const auto get_audiences = [parent, settings](YAML::Node yaml) {
    std::vector<YAML::Node> result;
    for (const auto & audiences : settings.audiences) {
      if (const auto node = yaml[audiences]) {
        result.push_back(node);
      } else {
        throw std::runtime_error(audiences + " audience is required in " + parent->path());
      }
    }
    return result;
  };
  const auto get_languages = [parent, settings](YAML::Node yaml) {
    std::vector<YAML::Node> result;
    for (const auto & languages : settings.languages) {
      if (const auto node = yaml[languages]) {
        result.push_back(node);
      } else {
        throw std::runtime_error(languages + " language is required in " + parent->path());
      }
    }
    return result;
  };
  const auto get_text = [parent](YAML::Node yaml, const std::string & field) {
    if (const auto node = yaml[field]) {
      return node.as<std::string>();
    } else {
      throw std::runtime_error(field + " field is required in " + parent->path());
    }
  };

  if (const auto node = yaml["condition"]) {
    condition_ = Condition::parse(node.as<std::string>());
  }

  if (const auto node = yaml["audiences"]) {
    for (const auto & audience : get_audiences(node)) {
      std::vector<std::string> situations;
      std::vector<std::string> solutions;
      for (const auto & language : get_languages(audience)) {
        situations.push_back(get_text(language, "situation"));
        solutions.push_back(get_text(language, "solution"));
      }
      situations_.push_back(situations);
      solutions_.push_back(solutions);
    }
  } else {
    throw std::runtime_error("audiences field is required in " + parent->path());
  }
}

Messages::Messages(const Notification * parent, const YAML::Node yaml, const Settings & settings)
{
  for (const auto & node : yaml) {
    entities_.emplace_back(std::make_unique<Message>(parent, node, settings));
  }

  for (const auto & entity : entities_) {
    pointers_.push_back(entity.get());
  }
}

}  // namespace autoware::failure_notification
