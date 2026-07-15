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

#include "monitoring.hpp"

namespace tier4_monitoring
{

Monitoring::Monitoring(const rclcpp::NodeOptions & options) : Node("monitoring", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  pub_status_ = create_publisher<MonitoringStatus>("~/status", rclcpp::QoS(1));
  sub_heartbeat_ = create_subscription<MonitoringHeartbeat>(
    "~/heartbeat", rclcpp::QoS(1), std::bind(&Monitoring::on_heartbeat, this, _1));
  srv_change_monitoring_mode_ = create_service<ChangeMonitoringMode>(
    "~/change", std::bind(&Monitoring::on_change_monitoring_mode, this, _1, _2));
}

void Monitoring::on_heartbeat(const MonitoringHeartbeat::SharedPtr msg)
{
  (void)msg;
}

void Monitoring::on_change_monitoring_mode(
  const ChangeMonitoringMode::Request::SharedPtr req,
  const ChangeMonitoringMode::Response::SharedPtr res)
{
  (void)req;
  res->status.code = ResponseStatus::SUCCESS;
}

}  // namespace tier4_monitoring

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_monitoring::Monitoring)
