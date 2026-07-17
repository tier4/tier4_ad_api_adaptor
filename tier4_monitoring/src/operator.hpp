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

#ifndef OPERATOR_HPP_
#define OPERATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/monitoring_heartbeat.hpp>
#include <tier4_external_api_msgs/msg/monitoring_status.hpp>
#include <tier4_external_api_msgs/srv/change_monitoring_mode.hpp>

#include <string>

namespace tier4_monitoring
{

class Operator
{
public:
  Operator(rclcpp::Node & node, const std::string & ns);

private:
  using MonitoringStatus = tier4_external_api_msgs::msg::MonitoringStatus;
  using MonitoringHeartbeat = tier4_external_api_msgs::msg::MonitoringHeartbeat;
  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;
  using ChangeMonitoringMode = tier4_external_api_msgs::srv::ChangeMonitoringMode;

  rclcpp::Publisher<MonitoringStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<MonitoringHeartbeat>::SharedPtr sub_heartbeat_;
  rclcpp::Service<ChangeMonitoringMode>::SharedPtr srv_change_;

  void on_heartbeat(const MonitoringHeartbeat::SharedPtr msg);
  void on_change(
    const ChangeMonitoringMode::Request::SharedPtr req,
    const ChangeMonitoringMode::Response::SharedPtr res);
};

}  // namespace tier4_monitoring

#endif  // OPERATOR_HPP_
