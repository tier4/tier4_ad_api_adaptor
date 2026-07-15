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

#ifndef MONITORING_HPP_
#define MONITORING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/monitoring_heartbeat.hpp>
#include <tier4_external_api_msgs/msg/monitoring_status.hpp>
#include <tier4_external_api_msgs/srv/change_monitoring_mode.hpp>

namespace tier4_monitoring
{

class Monitoring : public rclcpp::Node
{
public:
  explicit Monitoring(const rclcpp::NodeOptions & options);

private:
  using MonitoringStatus = tier4_external_api_msgs::msg::MonitoringStatus;
  using MonitoringHeartbeat = tier4_external_api_msgs::msg::MonitoringHeartbeat;
  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;
  using ChangeMonitoringMode = tier4_external_api_msgs::srv::ChangeMonitoringMode;

  rclcpp::Publisher<MonitoringStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<MonitoringHeartbeat>::SharedPtr sub_heartbeat_;
  rclcpp::Service<ChangeMonitoringMode>::SharedPtr srv_change_monitoring_mode_;

  void on_heartbeat(const MonitoringHeartbeat::SharedPtr msg);
  void on_change_monitoring_mode(
    const ChangeMonitoringMode::Request::SharedPtr req,
    const ChangeMonitoringMode::Response::SharedPtr res);
};

}  // namespace tier4_monitoring

#endif  // MONITORING_HPP_
