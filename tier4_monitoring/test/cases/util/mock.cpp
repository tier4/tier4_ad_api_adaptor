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

#include "mock.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

Client::Client(rclcpp::Node & node, const std::string & name)
{
  cli_change_ =
    node.create_client<ChangeMonitoringStatus>("/api/external/set/monitoring/" + name + "/change");
  pub_heartbeat_ = node.create_publisher<MonitoringHeartbeat>(
    "/api/external/set/monitoring/" + name + "/heartbeat", rclcpp::QoS(1));
  sub_status_ = node.create_subscription<MonitoringStatus>(
    "/api/external/get/monitoring/" + name + "/status", rclcpp::QoS(1),
    [this](const MonitoringStatus & msg) { status_ = msg; });
}

bool Client::is_ready() const
{
  if (!cli_change_->service_is_ready()) return false;
  if (pub_heartbeat_->get_subscription_count() == 0) return false;
  if (sub_status_->get_publisher_count() == 0) return false;
  return true;
}

void Client::heartbeat(const rclcpp::Time & stamp)
{
  MonitoringHeartbeat msg;
  msg.stamp = stamp;
  pub_heartbeat_->publish(msg);
}

Client::ChangeFuture Client::change(uint8_t status)
{
  const auto req = std::make_shared<ChangeMonitoringStatus::Request>();
  req->status = status;
  return cli_change_->async_send_request(req).future.share();
}

DrivingClient::DrivingClient(rclcpp::Node & node)
{
  cli_enable_ = node.create_client<EnableDriving>("/api/external/set/monitoring/driving/enable");
  sub_status_ = node.create_subscription<DrivingStatus>(
    "/api/external/get/monitoring/driving/status", rclcpp::QoS(1).transient_local(),
    [this](const DrivingStatus & msg) { status_ = msg; });
}

bool DrivingClient::is_ready() const
{
  if (!cli_enable_->service_is_ready()) return false;
  if (sub_status_->get_publisher_count() == 0) return false;
  return true;
}

DrivingClient::EnableFuture DrivingClient::enable(uint8_t mode)
{
  const auto req = std::make_shared<EnableDriving::Request>();
  req->mode = mode;
  return cli_enable_->async_send_request(req).future.share();
}

OperationMode::OperationMode(rclcpp::Node & node)
{
  clock_ = node.get_clock();
  pub_state_ = node.create_publisher<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local());
  const auto create_service = [&node, this](const std::string & name, uint8_t mode) {
    return node.create_service<ChangeOperationMode>(
      name, [this, mode](
              const ChangeOperationMode::Request::SharedPtr,
              const ChangeOperationMode::Response::SharedPtr res) { change(mode, res); });
  };
  srv_change_stop_mode_ =
    create_service("/api/operation_mode/change_to_stop", OperationModeState::STOP);
  srv_change_autonomous_mode_ =
    create_service("/api/operation_mode/change_to_autonomous", OperationModeState::AUTONOMOUS);

  state_.mode = OperationModeState::STOP;
  state_.is_autoware_control_enabled = true;
  state_.is_stop_mode_available = true;
  state_.is_autonomous_mode_available = true;
  publish();
}

bool OperationMode::is_ready() const
{
  return pub_state_->get_subscription_count() != 0;
}

void OperationMode::change(uint8_t mode, const ChangeOperationMode::Response::SharedPtr res)
{
  state_.mode = mode;
  publish();
  res->status.success = true;
}

void OperationMode::publish()
{
  state_.stamp = clock_->now();
  pub_state_->publish(state_);
}

VectorMap::VectorMap(rclcpp::Node & node)
{
  pub_map_ =
    node.create_publisher<LaneletMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local());

  const auto path = std::string(TEST_RESOURCE_PATH) + "/lanelet2.osm";
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(path);
  pub_map_->publish(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
}

bool VectorMap::is_ready() const
{
  return pub_map_->get_subscription_count() != 0;
}

Routing::Routing(rclcpp::Node & node)
{
  clock_ = node.get_clock();
  pub_route_ = node.create_publisher<Route>("/api/routing/route", rclcpp::QoS(1).transient_local());
}

bool Routing::is_ready() const
{
  return pub_route_->get_subscription_count() != 0;
}

void Routing::set_route(const std::vector<int64_t> & ids)
{
  Route msg;
  msg.header.stamp = clock_->now();
  msg.data.emplace_back();
  for (const auto & id : ids) {
    auto & segment = msg.data.front().segments.emplace_back();
    segment.preferred.id = id;
    segment.preferred.type = "lane";
  }
  pub_route_->publish(msg);
}

MockNode::MockNode() : rclcpp::Node("mock")
{
  names_ = {"supervisor/mot", "supervisor/remote", "advisor/mot", "advisor/remote"};
  for (const auto & name : names_) {
    clients_.emplace(name, std::make_shared<Client>(*this, name));
  }
  driving_ = std::make_shared<DrivingClient>(*this);
  operation_mode_ = std::make_shared<OperationMode>(*this);
  vector_map_ = std::make_shared<VectorMap>(*this);
  routing_ = std::make_shared<Routing>(*this);
}

bool MockNode::is_ready() const
{
  for (const auto & [name, client] : clients_) {
    if (!client->is_ready()) return false;
  }
  if (!driving_->is_ready()) return false;
  if (!operation_mode_->is_ready()) return false;
  if (!vector_map_->is_ready()) return false;
  if (!routing_->is_ready()) return false;
  return true;
}

void MockNode::heartbeat()
{
  const auto stamp = now();
  for (const auto & [name, client] : clients_) {
    client->heartbeat(stamp);
  }
}
