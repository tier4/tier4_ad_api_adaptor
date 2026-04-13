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

#include <memory>
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
: Node("maintenance_management", options), diagnostics_(this, 5.0)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto path = declare_parameter<std::string>("path");
  store_ = maintenance::Store(path);

  srv_set_state_ = create_service<SetState>(
    "/api/external/set/maintenance/state",
    std::bind(&MaintenanceManagement::on_set_state, this, _1, _2));
  srv_get_state_ = create_service<GetState>(
    "/api/external/get/maintenance/state",
    std::bind(&MaintenanceManagement::on_get_state, this, _1, _2));
  sub_operation_mode_ = create_subscription<OperationMode>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    [this](const OperationMode & msg) { operation_mode_ = msg; });

  diagnostics_.setHardwareID("none");
  diagnostics_.add("state", [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
    const auto state = store_.read();
    if (state == maintenance::State::OFF) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, state_text(state));
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, state_text(state));
    }
  });

  operation_mode_.mode = OperationMode::UNKNOWN;
}

void MaintenanceManagement::on_get_state(
  const std::shared_ptr<rmw_request_id_t> header, const GetState::Request::SharedPtr)
{
  using tier4_external_api_msgs::msg::ResponseStatus;
  GetState::Response res;

  switch (store_.read()) {
    case maintenance::State::ON:
      res.maintenance = true;
      res.status.code = ResponseStatus::SUCCESS;
      break;
    case maintenance::State::OFF:
      res.maintenance = false;
      res.status.code = ResponseStatus::SUCCESS;
      break;
    default:
      res.maintenance = false;
      res.status.code = ResponseStatus::ERROR;
      res.status.message = "unknown state";
      break;
  }

  srv_get_state_->send_response(*header, res);
}

void MaintenanceManagement::on_set_state(
  const std::shared_ptr<rmw_request_id_t> header, const SetState::Request::SharedPtr req)
{
  using tier4_external_api_msgs::msg::ResponseStatus;
  SetState::Response res;

  if (operation_mode_.mode != OperationMode::STOP) {
    res.status.code = ResponseStatus::ERROR;
    res.status.message = "operation mode is not stop";
    return srv_set_state_->send_response(*header, res);
  }

  const auto state = req->maintenance ? maintenance::State::ON : maintenance::State::OFF;
  if (!store_.write(state)) {
    res.status.code = ResponseStatus::ERROR;
    res.status.message = "failed to write state";
    return srv_set_state_->send_response(*header, res);
  }

  diagnostics_.force_update();

  res.status.code = ResponseStatus::SUCCESS;
  return srv_set_state_->send_response(*header, res);
}

}  // namespace tier4_maintenance_management

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_maintenance_management::MaintenanceManagement)
