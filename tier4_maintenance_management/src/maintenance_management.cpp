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

#include "maintenance_management.hpp"

#include <string>

namespace tier4_maintenance_management
{

std::string state_text(const maintenance::State & state)
{
  switch (state) {
    case maintenance::State::UNKNOWN:
      return "UNKNOWN";
    case maintenance::State::ON:
      return "ON";
    case maintenance::State::OFF:
      return "OFF";
    default:
      return "INVALID";
  }
}

MaintenanceManagement::MaintenanceManagement(const rclcpp::NodeOptions & options)
: Node("maintenance_management", options)
{
  const auto path = declare_parameter<std::string>("path");
  store_ = maintenance::Store(path);
  RCLCPP_INFO_STREAM(get_logger(), "Store Path: " << path);

  const auto state = store_.read();
  RCLCPP_INFO_STREAM(get_logger(), "State: " << state_text(state));

  const auto ok = store_.write(maintenance::State::ON);
  RCLCPP_INFO_STREAM(get_logger(), "Write OK: " << ok);
}

void MaintenanceManagement::on_get_state(
  const GetState::Request::SharedPtr req, const GetState::Response::SharedPtr res)
{
  // lock
  (void)req;
  (void)res;
}

void MaintenanceManagement::on_set_state(
  const SetState::Request::SharedPtr req, const SetState::Response::SharedPtr res)
{
  // lock
  (void)req;
  (void)res;
}

}  // namespace tier4_maintenance_management

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_maintenance_management::MaintenanceManagement)
