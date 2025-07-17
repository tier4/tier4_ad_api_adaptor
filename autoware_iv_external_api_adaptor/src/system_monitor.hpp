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

#include <rclcpp/rclcpp.hpp>
#include "tier4_api_utils/tier4_api_utils.hpp"

#include "tier4_external_api_msgs/msg/system_monitor.hpp"
#include "tier4_external_api_msgs/msg/cpu_temperature.hpp"
#include "tier4_external_api_msgs/msg/memory_status.hpp"
#include "tier4_external_api_msgs/msg/gpu_status.hpp"
#include "tier4_external_api_msgs/msg/network_status.hpp"
#include "tier4_external_api_msgs/msg/hdd_status.hpp"

#include <memory>
#include <utility>

namespace external_api
{

class SystemMonitor : public rclcpp::Node
{
public:
  explicit SystemMonitor(const rclcpp::NodeOptions & options);

private:
  void callbackTimer();

  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, tier4_external_api_msgs::msg::SystemMonitor> msg_system_monitor_;

  rclcpp::Publisher<tier4_external_api_msgs::msg::SystemMonitor>::SharedPtr pub_system_monitor_;

  rclcpp::Subscription<tier4_external_api_msgs::msg::CpuTemperature>::SharedPtr sub_cpu_temperature_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::MemoryStatus>::SharedPtr sub_memory_status_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::GpuStatus>::SharedPtr sub_gpu_status_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::NetworkStatus>::SharedPtr sub_network_status_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::HddStatus>::SharedPtr sub_hdd_status_;
};

}  // namespace external_api

#endif  // SYSTEM_MONITOR_HPP_
