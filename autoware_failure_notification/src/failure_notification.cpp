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
  const auto path = declare_parameter<std::string>("message");
  notifications_ = std::make_unique<Notifications>(path);

  // Set a non-existent pattern to ensure the first message is published.
  previous_messages_.push_back(nullptr);

  /*
    for (const auto & notification : notifications_->notifications()) {
      RCLCPP_INFO_STREAM(get_logger(), notification->path());
      RCLCPP_INFO_STREAM(get_logger(), "  priority: " << notification->priority());
      RCLCPP_INFO_STREAM(get_logger(), "  messages:");
      for (const auto & message : notification->messages()) {
        RCLCPP_INFO_STREAM(get_logger(), "    " << message->text());
      }
    }
  */

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
    RCLCPP_INFO_STREAM(get_logger(), "Node: " << node->path());
    mapping_[node] = dictionary[node->path()];
  }
}

void FailureNotification::on_update(DiagGraph::ConstSharedPtr graph)
{
  Context context;

  for (const auto & node : graph->nodes()) {
    const auto notification = mapping_.at(node);
    if (notification) {
      notification->update(context, node->level());
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
    RCLCPP_INFO_STREAM(get_logger(), "Messages updated");
    for (const auto & message : messages) {
      RCLCPP_INFO_STREAM(get_logger(), "  " << message->text());
    }
    previous_messages_ = messages;
  }
}

}  // namespace autoware::failure_notification

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::failure_notification::FailureNotification)
