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

#ifndef CORE__NOTIFICATION_HPP_
#define CORE__NOTIFICATION_HPP_

#include "message.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace autoware::failure_notification
{

class Notification
{
public:
  Notification(const std::string & path, YAML::Node yaml);
  const auto & path() const { return path_; }
  const auto & priority() const { return priority_; }
  const auto & messages() const { return messages_.messages(); }
  const auto & current_message() const { return current_message_; }
  const auto & current_level() const { return current_level_; }

  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagLevel = DiagStatus::_level_type;
  void update(const Context & context, DiagLevel level);

private:
  const std::string path_;
  int priority_;
  Messages messages_;

  const Message * current_message_ = nullptr;
  DiagLevel current_level_ = DiagStatus::OK;
};

class Notifications
{
public:
  explicit Notifications(const std::string & path);
  const auto & notifications() const { return pointers_; }

private:
  std::vector<std::unique_ptr<Notification>> entities_;
  std::vector<Notification *> pointers_;
};

}  // namespace autoware::failure_notification

#endif  // CORE__NOTIFICATION_HPP_
