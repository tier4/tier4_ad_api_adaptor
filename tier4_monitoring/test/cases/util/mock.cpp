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

#include <memory>
#include <string>

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

MockNode::MockNode() : rclcpp::Node("mock")
{
  names_ = {"supervisor/mot", "supervisor/remote", "advisor/mot", "advisor/remote"};
  for (const auto & name : names_) {
    clients_.emplace(name, std::make_shared<Client>(*this, name));
  }
  driving_ = std::make_shared<DrivingClient>(*this);
}

bool MockNode::is_ready() const
{
  for (const auto & [name, client] : clients_) {
    if (!client->is_ready()) return false;
  }
  if (!driving_->is_ready()) return false;
  return true;
}

void MockNode::heartbeat()
{
  const auto stamp = now();
  for (const auto & [name, client] : clients_) {
    client->heartbeat(stamp);
  }
}
