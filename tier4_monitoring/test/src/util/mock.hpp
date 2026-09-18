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

#ifndef UTIL__MOCK_HPP_
#define UTIL__MOCK_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/monitoring_heartbeat.hpp>
#include <tier4_external_api_msgs/msg/monitoring_status.hpp>
#include <tier4_external_api_msgs/srv/change_monitoring_status.hpp>

#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using tier4_external_api_msgs::msg::MonitoringHeartbeat;
using tier4_external_api_msgs::msg::MonitoringStatus;
using tier4_external_api_msgs::srv::ChangeMonitoringStatus;

class Client
{
public:
  using ChangeFuture = std::shared_future<ChangeMonitoringStatus::Response::SharedPtr>;

  Client(rclcpp::Node & node, const std::string & name);
  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;

  bool is_ready() const;
  void heartbeat(const rclcpp::Time & stamp);
  ChangeFuture change(uint8_t status);
  const std::optional<MonitoringStatus> & status() const { return status_; }

private:
  rclcpp::Client<ChangeMonitoringStatus>::SharedPtr cli_change_;
  rclcpp::Publisher<MonitoringHeartbeat>::SharedPtr pub_heartbeat_;
  rclcpp::Subscription<MonitoringStatus>::SharedPtr sub_status_;
  std::optional<MonitoringStatus> status_;
};

class MockNode : public rclcpp::Node
{
public:
  MockNode();
  bool is_ready() const;
  void heartbeat();
  auto client(const std::string & name) { return clients_.at(name); }

private:
  std::vector<std::string> names_;
  std::unordered_map<std::string, std::shared_ptr<Client>> clients_;
};

#endif  // UTIL__MOCK_HPP_
