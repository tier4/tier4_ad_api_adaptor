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

#include "mock.hpp"

MockNode::MockNode() : rclcpp::Node("mock")
{
  sub_failure_notification_ = create_subscription<FailureNotificationArray>(
    "/api/external/get/failure_notification", rclcpp::QoS(1).transient_local(),
    [this](const FailureNotificationArray & msg) { notifications.push_back(msg); });

  pub_graph_struct_ = create_publisher<DiagGraphStruct>(
    "/diagnostics_graph/struct", rclcpp::QoS(1).transient_local());
  pub_graph_status_ =
    create_publisher<DiagGraphStatus>("/diagnostics_graph/status", rclcpp::QoS(1));
}

void MockNode::publish_struct(const DiagGraphStruct & msg)
{
  pub_graph_struct_->publish(msg);
}

void MockNode::publish_status(const DiagGraphStatus & msg)
{
  pub_graph_status_->publish(msg);
}
