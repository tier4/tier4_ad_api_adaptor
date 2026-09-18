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

#include <string>

Client::Client(rclcpp::Node & node, const std::string & name)
{
  cli_change_ =
    node.create_client<ChangeMonitoringStatus>("/api/external/set/monitoring/" + name + "/change");
  pub_heartbeat_ = node.create_publisher<MonitoringHeartbeat>(
    "/api/external/set/monitoring/" + name + "/heartbeat", rclcpp::QoS(1));
  sub_status_ = node.create_subscription<MonitoringStatus>(
    "/api/external/get/monitoring/" + name + "/status", rclcpp::QoS(1),
    [this](const MonitoringStatus & msg) { status_ = msg; });
}

MockNode::MockNode() : rclcpp::Node("mock")
{
  clients_.push_back(Client(*this, "supervisor/mot"));
  clients_.push_back(Client(*this, "supervisor/remote"));
  clients_.push_back(Client(*this, "advisor/mot"));
  clients_.push_back(Client(*this, "advisor/remote"));
}
