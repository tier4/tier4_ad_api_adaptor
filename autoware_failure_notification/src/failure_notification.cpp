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

#include "failure_notification.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::failure_notification
{

FailureNotification::FailureNotification(const rclcpp::NodeOptions & options)
: Node("failure_notification", options)
{
  settings_.audiences = declare_parameter<std::vector<std::string>>("audiences");
  settings_.languages = declare_parameter<std::vector<std::string>>("languages");

  const auto path = declare_parameter<std::string>("message");
  notifications_ = std::make_unique<Notifications>(YAML::LoadFile(path), settings_);

  // Set a non-existent pattern to ensure the first message is published.
  previous_messages_.push_back(nullptr);

  // Context.
  context_.route_state.stamp = now();
  context_.route_state.state = Context::RouteState::UNKNOWN;
  context_.localization_state.stamp = now();
  context_.localization_state.state = Context::LocalizationState::UNKNOWN;
  sub_route_state_ = create_subscription<Context::RouteState>(
    "/api/routing/state", rclcpp::QoS(1).transient_local(),
    [this](const Context::RouteState & msg) { context_.route_state = msg; });
  sub_localization_state_ = create_subscription<Context::LocalizationState>(
    "/api/localization/initialization_state", rclcpp::QoS(1).transient_local(),
    [this](const Context::LocalizationState & msg) { context_.localization_state = msg; });

  for (const auto & audience : settings_.audiences) {
    pub_failure_notification_.push_back(
      create_publisher<FailureNotificationArray>(
        "/system/failure_notification/" + audience, rclcpp::QoS(1).best_effort()));
  }

  using std::placeholders::_1;
  sub_graph_.register_create_callback(std::bind(&FailureNotification::on_create, this, _1));
  sub_graph_.register_update_callback(std::bind(&FailureNotification::on_update, this, _1));
  sub_graph_.subscribe(*this, 1);
}

void FailureNotification::on_create(DiagGraph::ConstSharedPtr graph)
{
  std::unordered_map<std::string, Notification *> dictionary;
  for (const auto & notification : notifications_->notifications()) {
    dictionary[notification->path()] = notification;
  }
  for (const auto & node : graph->nodes()) {
    mapping_[node] = dictionary[node->path()];
  }
}

void FailureNotification::on_update(DiagGraph::ConstSharedPtr graph)
{
  for (const auto & node : graph->nodes()) {
    const auto notification = mapping_.at(node);
    if (notification) {
      notification->update(context_, node->level());
    }
  }

  std::vector<const Message *> messages;
  for (const auto & notification : notifications_->notifications()) {
    const auto message = notification->current_message();
    if (message) {
      messages.push_back(message);
    }
  }

  if (previous_messages_ != messages) {
    for (size_t audiences = 0; audiences < settings_.audiences.size(); ++audiences) {
      FailureNotificationArray msg;
      for (const auto & message : messages) {
        FailureNotificationMsg notification;
        notification.language_codes = settings_.languages;
        notification.situations = message->situations(audiences);
        notification.solutions = message->solutions(audiences);
        msg.notifications.push_back(notification);
      }
      pub_failure_notification_.at(audiences)->publish(msg);
    }
    previous_messages_ = messages;
  }
}

}  // namespace autoware::failure_notification

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::failure_notification::FailureNotification)
