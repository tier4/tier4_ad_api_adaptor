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

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tier4_external_api_msgs/srv/get_maintenance_mode.hpp>
#include <tier4_external_api_msgs/srv/set_maintenance_mode.hpp>

#include <memory>
#include <mutex>

namespace tier4_maintenance_management
{

class MaintenanceManagement : public rclcpp::Node
{
public:
  explicit MaintenanceManagement(const rclcpp::NodeOptions & options);

private:
  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;
  using SetMode = tier4_external_api_msgs::srv::SetMaintenanceMode;
  using GetMode = tier4_external_api_msgs::srv::GetMaintenanceMode;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Service<SetMode>::SharedPtr srv_set_mode_;
  rclcpp::Service<GetMode>::SharedPtr srv_get_mode_;
  void on_get_mode(
    const std::shared_ptr<rmw_request_id_t> header, const GetMode::Request::SharedPtr req);
  void on_set_mode(
    const std::shared_ptr<rmw_request_id_t> header, const SetMode::Request::SharedPtr req);

  rclcpp::TimerBase::SharedPtr mode_on_timer_;
  void set_mode_off(const std::shared_ptr<rmw_request_id_t> header);
  void set_mode_on1(const std::shared_ptr<rmw_request_id_t> header);
  void set_mode_on2(const std::shared_ptr<rmw_request_id_t> header);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diagnostics_;
  void on_timer();
  void publish_diagnostics();

  double operation_mode_check_duration_;
  bool is_maintenance_requesting_;
  maintenance::Store store_;
  OperationModeState operation_mode_;
  std::mutex mutex_;
};

}  // namespace tier4_maintenance_management

#endif  // MAINTENANCE_MANAGEMENT_HPP_
