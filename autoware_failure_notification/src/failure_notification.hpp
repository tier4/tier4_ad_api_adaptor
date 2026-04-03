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

#ifndef FAILURE_NOTIFICATION_HPP_
#define FAILURE_NOTIFICATION_HPP_

#include <autoware/diagnostic_graph_utils/subscription.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::failure_notification
{

class FailureNotification : public rclcpp::Node
{
public:
  explicit Converter(const rclcpp::NodeOptions & options);

private:
  using DiagGraph = autoware::diagnostic_graph_utils::DiagGraph;
  using DiagUnit = autoware::diagnostic_graph_utils::DiagUnit;
  using DiagNode = autoware::diagnostic_graph_utils::DiagNode;
  void on_create(DiagGraph::ConstSharedPtr graph);
  void on_update(DiagGraph::ConstSharedPtr graph);
  autoware::diagnostic_graph_utils::DiagGraphSubscription sub_graph_;
};

}  // namespace autoware::failure_notification

#endif  // FAILURE_NOTIFICATION_HPP_
