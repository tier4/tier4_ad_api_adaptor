// Copyright 2026 TIER IV, Inc.
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

#include "operator.hpp"

#include <string>

namespace tier4_monitoring
{

Operator::Operator(rclcpp::Node & node, const std::string & ns)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  pub_status_ = node.create_publisher<MonitoringStatus>(ns + "/status", rclcpp::QoS(1));
  sub_heartbeat_ = node.create_subscription<MonitoringHeartbeat>(
    ns + "/heartbeat", rclcpp::QoS(1), std::bind(&Operator::on_heartbeat, this, _1));
  srv_change_ = node.create_service<ChangeMonitoringMode>(
    ns + "/change", std::bind(&Operator::on_change, this, _1, _2));
}

void Operator::on_heartbeat(const MonitoringHeartbeat::SharedPtr msg)
{
  (void)msg;
}

void Operator::on_change(
  const ChangeMonitoringMode::Request::SharedPtr req,
  const ChangeMonitoringMode::Response::SharedPtr res)
{
  (void)req;
  res->status.code = ResponseStatus::SUCCESS;
}

}  // namespace tier4_monitoring
