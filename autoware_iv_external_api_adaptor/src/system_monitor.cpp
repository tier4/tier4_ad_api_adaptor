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

#include <string>

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
    [this](const tier4_external_api_msgs::msg::CpuTemperature::ConstSharedPtr msg) {
      msg_system_monitor_[msg->hostname].cpu_temperature = *msg;
      tryPublishMsg(msg->hostname);
    });

  sub_memory_status_ = create_subscription<tier4_external_api_msgs::msg::MemoryStatus>(
    "/system/system_monitor/mem_monitor/memory_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::MemoryStatus::ConstSharedPtr msg) {
      msg_system_monitor_[msg->hostname].memory_status = *msg;
      tryPublishMsg(msg->hostname);
    });

  sub_gpu_status_ = create_subscription<tier4_external_api_msgs::msg::GpuStatus>(
    "/system/system_monitor/gpu_monitor/gpu_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::GpuStatus::ConstSharedPtr msg) {
      msg_system_monitor_[msg->hostname].gpu_status = *msg;
      tryPublishMsg(msg->hostname);
    });

  sub_network_status_ = create_subscription<tier4_external_api_msgs::msg::NetworkStatus>(
    "/system/system_monitor/net_monitor/network_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::NetworkStatus::ConstSharedPtr msg) {
      msg_system_monitor_[msg->hostname].network_status = *msg;
      tryPublishMsg(msg->hostname);
    });

  sub_hdd_status_ = create_subscription<tier4_external_api_msgs::msg::HddStatus>(
    "/system/system_monitor/hdd_monitor/hdd_status", rclcpp::QoS(1),
    [this](const tier4_external_api_msgs::msg::HddStatus::ConstSharedPtr msg) {
      msg_system_monitor_[msg->hostname].hdd_status = *msg;
      tryPublishMsg(msg->hostname);
    });
}

bool SystemMonitor::isComplete(const tier4_external_api_msgs::msg::SystemMonitor & msg)
{
  // Checks whether all fields in the SystemMonitor message are filled.
  // The timestamp fields are initialized to zero by the default constructor,
  // so a non-zero value indicates that the corresponding message was received.
  return msg.cpu_temperature.stamp.sec != 0 && msg.memory_status.stamp.sec != 0 &&
         msg.gpu_status.stamp.sec != 0 && msg.network_status.stamp.sec != 0 &&
         msg.hdd_status.stamp.sec != 0;
}

void SystemMonitor::tryPublishMsg(const std::string & hostname)
{
  auto it = msg_system_monitor_.find(hostname);
  if (it == msg_system_monitor_.end()) return;

  auto & msg = it->second;
  if (isComplete(msg)) {
    msg.hostname = hostname;
    msg.stamp = this->now();
    pub_system_monitor_->publish(msg);
    msg_system_monitor_.erase(it);
  }
}

}  // namespace external_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::SystemMonitor)
