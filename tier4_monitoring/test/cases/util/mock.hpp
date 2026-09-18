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

#ifndef CASES__UTIL__MOCK_HPP_
#define CASES__UTIL__MOCK_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tier4_external_api_msgs/msg/driving_status.hpp>
#include <tier4_external_api_msgs/msg/monitoring_heartbeat.hpp>
#include <tier4_external_api_msgs/msg/monitoring_status.hpp>
#include <tier4_external_api_msgs/srv/change_monitoring_status.hpp>
#include <tier4_external_api_msgs/srv/enable_driving.hpp>

#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_adapi_v1_msgs::msg::Route;
using autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;
using autoware_map_msgs::msg::LaneletMapBin;
using tier4_external_api_msgs::msg::DrivingStatus;
using tier4_external_api_msgs::msg::MonitoringHeartbeat;
using tier4_external_api_msgs::msg::MonitoringStatus;
using tier4_external_api_msgs::srv::ChangeMonitoringStatus;
using tier4_external_api_msgs::srv::EnableDriving;

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

class DrivingClient
{
public:
  using EnableFuture = std::shared_future<EnableDriving::Response::SharedPtr>;

  explicit DrivingClient(rclcpp::Node & node);
  DrivingClient(const DrivingClient &) = delete;
  DrivingClient & operator=(const DrivingClient &) = delete;

  bool is_ready() const;
  EnableFuture enable(uint8_t mode);
  const std::optional<DrivingStatus> & status() const { return status_; }

private:
  rclcpp::Client<EnableDriving>::SharedPtr cli_enable_;
  rclcpp::Subscription<DrivingStatus>::SharedPtr sub_status_;
  std::optional<DrivingStatus> status_;
};

class OperationMode
{
public:
  explicit OperationMode(rclcpp::Node & node);
  OperationMode(const OperationMode &) = delete;
  OperationMode & operator=(const OperationMode &) = delete;

  bool is_ready() const;
  const OperationModeState & state() const { return state_; }

private:
  void change(uint8_t mode, const ChangeOperationMode::Response::SharedPtr res);
  void publish();

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_state_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_change_stop_mode_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_change_autonomous_mode_;
  OperationModeState state_;
};

// Dummy vector map that publishes the lanelet2 map in the test resource directory.
class VectorMap
{
public:
  explicit VectorMap(rclcpp::Node & node);
  VectorMap(const VectorMap &) = delete;
  VectorMap & operator=(const VectorMap &) = delete;

  bool is_ready() const;

private:
  rclcpp::Publisher<LaneletMapBin>::SharedPtr pub_map_;
};

// Dummy routing that publishes a route consisting of the given lanelets.
class Routing
{
public:
  explicit Routing(rclcpp::Node & node);
  Routing(const Routing &) = delete;
  Routing & operator=(const Routing &) = delete;

  bool is_ready() const;
  void set_route(const std::vector<int64_t> & ids);

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<Route>::SharedPtr pub_route_;
};

// Dummy planning that records the velocity limit set and clear requests.
class Planning
{
public:
  explicit Planning(rclcpp::Node & node);
  Planning(const Planning &) = delete;
  Planning & operator=(const Planning &) = delete;

  bool is_ready() const;
  void reset();
  const std::optional<VelocityLimit> & velocity_limit_set() const { return set_; }
  const std::optional<VelocityLimitClearCommand> & velocity_limit_clear() const { return clear_; }

private:
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_set_;
  rclcpp::Subscription<VelocityLimitClearCommand>::SharedPtr sub_clear_;
  std::optional<VelocityLimit> set_;
  std::optional<VelocityLimitClearCommand> clear_;
};

class MockNode : public rclcpp::Node
{
public:
  MockNode();
  bool is_ready() const;
  void heartbeat();
  auto client(const std::string & name) { return clients_.at(name); }
  auto driving() { return driving_; }
  auto operation_mode() { return operation_mode_; }
  auto routing() { return routing_; }
  auto planning() { return planning_; }

private:
  std::vector<std::string> names_;
  std::unordered_map<std::string, std::shared_ptr<Client>> clients_;
  std::shared_ptr<DrivingClient> driving_;
  std::shared_ptr<OperationMode> operation_mode_;
  std::shared_ptr<VectorMap> vector_map_;
  std::shared_ptr<Routing> routing_;
  std::shared_ptr<Planning> planning_;
};

#endif  // CASES__UTIL__MOCK_HPP_
