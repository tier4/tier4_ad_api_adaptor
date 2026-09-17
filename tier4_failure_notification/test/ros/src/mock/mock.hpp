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

#ifndef MOCK__MOCK_HPP_
#define MOCK__MOCK_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/failure_notification_array.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>

#include <vector>

using FailureNotificationArray = tier4_external_api_msgs::msg::FailureNotificationArray;
using FailureNotificationMsg = tier4_external_api_msgs::msg::FailureNotification;
using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
using DiagNodeStruct = tier4_system_msgs::msg::DiagNodeStruct;
using DiagNodeStatus = tier4_system_msgs::msg::DiagNodeStatus;

class MockNode : public rclcpp::Node
{
public:
  MockNode();
  void publish_struct(const DiagGraphStruct & msg);
  void publish_status(const DiagGraphStatus & msg);

  std::vector<FailureNotificationArray> notifications;

private:
  rclcpp::Subscription<FailureNotificationArray>::SharedPtr sub_failure_notification_;
  rclcpp::Publisher<DiagGraphStruct>::SharedPtr pub_graph_struct_;
  rclcpp::Publisher<DiagGraphStatus>::SharedPtr pub_graph_status_;
};

#endif  // MOCK__MOCK_HPP_
