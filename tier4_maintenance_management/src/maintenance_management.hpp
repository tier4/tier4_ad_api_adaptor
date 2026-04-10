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

#ifndef MAINTENANCE_MANAGEMENT_HPP_
#define MAINTENANCE_MANAGEMENT_HPP_

#include <maintenance_state_store/maintenance_state_store.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/srv/get_maintenance_state.hpp>
#include <tier4_external_api_msgs/srv/set_maintenance_state.hpp>

namespace tier4_maintenance_management
{

class MaintenanceManagement : public rclcpp::Node
{
public:
  explicit MaintenanceManagement(const rclcpp::NodeOptions & options);

private:
  using SetState = tier4_external_api_msgs::srv::SetMaintenanceState;
  using GetState = tier4_external_api_msgs::srv::GetMaintenanceState;

  rclcpp::Service<SetState>::SharedPtr srv_set_state_;
  rclcpp::Service<GetState>::SharedPtr srv_get_state_;
  // Operation Mode

  void on_get_state(
    const GetState::Request::SharedPtr req, const GetState::Response::SharedPtr res);
  void on_set_state(
    const SetState::Request::SharedPtr req, const SetState::Response::SharedPtr res);

  maintenance::Store store_;
};

}  // namespace tier4_maintenance_management

#endif  // MAINTENANCE_MANAGEMENT_HPP_
