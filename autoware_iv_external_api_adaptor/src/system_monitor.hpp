// Copyright 2025 TIER IV, Inc.
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

#ifndef SYSTEM_MONITOR_HPP_
#define SYSTEM_MONITOR_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tier4_external_api_msgs/msg/cpu_temperature.hpp"
#include "tier4_external_api_msgs/msg/gpu_status.hpp"
#include "tier4_external_api_msgs/msg/hdd_status.hpp"
#include "tier4_external_api_msgs/msg/memory_status.hpp"
#include "tier4_external_api_msgs/msg/network_status.hpp"
#include "tier4_external_api_msgs/msg/system_monitor.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace external_api
{

class SystemMonitor : public autoware::agnocast_wrapper::Node
{
public:
  explicit SystemMonitor(const rclcpp::NodeOptions & options);

private:
  bool isComplete(const tier4_external_api_msgs::msg::SystemMonitor & msg);
  void tryPublishMsg(const std::string & hostname);

  std::map<std::string, tier4_external_api_msgs::msg::SystemMonitor> msg_system_monitor_;

  AUTOWARE_PUBLISHER_PTR(tier4_external_api_msgs::msg::SystemMonitor) pub_system_monitor_;

  AUTOWARE_SUBSCRIPTION_PTR(tier4_external_api_msgs::msg::CpuTemperature) sub_cpu_temperature_;
  AUTOWARE_SUBSCRIPTION_PTR(tier4_external_api_msgs::msg::MemoryStatus) sub_memory_status_;
  AUTOWARE_SUBSCRIPTION_PTR(tier4_external_api_msgs::msg::GpuStatus) sub_gpu_status_;
  AUTOWARE_SUBSCRIPTION_PTR(tier4_external_api_msgs::msg::NetworkStatus) sub_network_status_;
  AUTOWARE_SUBSCRIPTION_PTR(tier4_external_api_msgs::msg::HddStatus) sub_hdd_status_;
};

}  // namespace external_api

#endif  // SYSTEM_MONITOR_HPP_
