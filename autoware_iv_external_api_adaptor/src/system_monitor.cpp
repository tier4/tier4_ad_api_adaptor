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

#include "system_monitor.hpp"

namespace external_api
{

SystemMonitor::SystemMonitor(const rclcpp::NodeOptions & options) : Node("system_monitor", options)
{
  using namespace std::literals::chrono_literals;

  // Publisher
  pub_system_monitor_ = this->create_publisher<tier4_external_api_msgs::msg::SystemMonitor>(
    "/api/external/get/system_monitor", rclcpp::QoS(1));

  // Subscriber
  sub_cpu_temperature_ = create_subscription<tier4_external_api_msgs::msg::CpuTemperature>(
    "/system/system_monitor/cpu_monitor/cpu_temperature", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::CpuTemperature::SharedPtr msg) {
      msg_system_monitor_[msg->hostname].cpu_temperature = *msg;
    });

  sub_memory_status_ = create_subscription<tier4_external_api_msgs::msg::MemoryStatus>(
    "/system/system_monitor/mem_monitor/memory_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::MemoryStatus::SharedPtr msg) {
      msg_system_monitor_[msg->hostname].memory_status = *msg;
    });

  sub_gpu_status_ = create_subscription<tier4_external_api_msgs::msg::GpuStatus>(
    "/system/system_monitor/gpu_monitor/gpu_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::GpuStatus::SharedPtr msg) {
      msg_system_monitor_[msg->hostname].gpu_status = *msg;
    });

  sub_network_status_ = create_subscription<tier4_external_api_msgs::msg::NetworkStatus>(
    "/system/system_monitor/net_monitor/network_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::NetworkStatus::SharedPtr msg) {
      msg_system_monitor_[msg->hostname].network_status = *msg;
    });

  sub_hdd_status_ = create_subscription<tier4_external_api_msgs::msg::HddStatus>(
    "/system/system_monitor/hdd_monitor/hdd_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::HddStatus::SharedPtr msg) {
      msg_system_monitor_[msg->hostname].hdd_status = *msg;
    });

  // Timer callback
  timer_ =
    rclcpp::create_timer(this, get_clock(), 1s, std::bind(&SystemMonitor::callbackTimer, this));
}

void SystemMonitor::callbackTimer()
{
  for (auto & [hostname, msg] : msg_system_monitor_) {
    msg.hostname = hostname;
    msg.stamp = this->now();
    pub_system_monitor_->publish(msg);
  }
}

}  // namespace external_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::SystemMonitor)
