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

MaintenanceManagement::MaintenanceManagement(const rclcpp::NodeOptions & options)
: Node("maintenance_management", options), store_(declare_parameter<std::string>("path"))
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  operation_mode_check_duration_ = declare_parameter<double>("operation_mode_check_duration");
  is_maintenance_requesting_ = false;
  operation_mode_.stamp = now();
  operation_mode_.mode = OperationModeState::UNKNOWN;

  srv_set_mode_ = create_service<SetMode>(
    "/api/external/set/maintenance/mode",
    std::bind(&MaintenanceManagement::on_set_mode, this, _1, _2));
  srv_get_mode_ = create_service<GetMode>(
    "/api/external/get/maintenance/mode",
    std::bind(&MaintenanceManagement::on_get_mode, this, _1, _2));
  sub_operation_mode_ = create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    [this](const OperationModeState & msg) { operation_mode_ = msg; });

  // The diagnostic_updater cannot be used because it publishes OK level at initialization.
  const auto period = rclcpp::Duration::from_seconds(1.0);
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
  pub_diagnostics_ = create_publisher<DiagnosticArray>("/diagnostics", rclcpp::QoS(1));
}

void MaintenanceManagement::on_get_mode(
  const std::shared_ptr<rmw_request_id_t> header, const GetMode::Request::SharedPtr)
{
  using MaintenanceMode = GetMode::Response::_mode_type;

  GetMode::Response res;
  switch (store_.read()) {
    case maintenance::State::ON:
      res.mode.state = MaintenanceMode::ON;
      res.status.code = ResponseStatus::SUCCESS;
      break;
    case maintenance::State::OFF:
      res.mode.state = MaintenanceMode::OFF;
      res.status.code = ResponseStatus::SUCCESS;
      break;
    default:
      res.mode.state = MaintenanceMode::UNKNOWN;
      res.status.code = ResponseStatus::SUCCESS;
      break;
  }
  srv_get_mode_->send_response(*header, res);
}

void MaintenanceManagement::on_set_mode(
  const std::shared_ptr<rmw_request_id_t> header, const SetMode::Request::SharedPtr req)
{
  using MaintenanceMode = SetMode::Request::_mode_type;

  const auto send_error_response = [this, header](const std::string & message) {
    SetMode::Response res;
    res.status.code = ResponseStatus::ERROR;
    res.status.message = message;
    srv_set_mode_->send_response(*header, res);
  };

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  if (!lock.try_lock()) {
    return send_error_response("another request is being processed");
  }
  if (req->mode.state != MaintenanceMode::ON && req->mode.state != MaintenanceMode::OFF) {
    return send_error_response("unknown mode requested");
  }
  if (store_.read() == maintenance::State::UNKNOWN) {
    return send_error_response("unknown state");
  }
  if (operation_mode_.mode != OperationModeState::STOP) {
    return send_error_response("operation mode is not stop");
  }

  if (req->mode.state == MaintenanceMode::ON) {
    lock.release();
    return set_mode_on1(header);
  }
  if (req->mode.state == MaintenanceMode::OFF) {
    lock.release();
    return set_mode_off(header);
  }
  throw std::logic_error("unreachable");
}

void MaintenanceManagement::set_mode_off(const std::shared_ptr<rmw_request_id_t> header)
{
  std::unique_lock<std::mutex> lock(mutex_, std::adopt_lock);

  SetMode::Response res;
  if (!store_.write(maintenance::State::OFF)) {
    res.status.code = ResponseStatus::ERROR;
    res.status.message = "failed to write state";
  } else {
    res.status.code = ResponseStatus::SUCCESS;
  }
  publish_diagnostics();
  srv_set_mode_->send_response(*header, res);
}

void MaintenanceManagement::set_mode_on1(const std::shared_ptr<rmw_request_id_t> header)
{
  is_maintenance_requesting_ = true;
  publish_diagnostics();

  mode_on_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(operation_mode_check_duration_),
    [this, header]() { set_mode_on2(header); });
}

void MaintenanceManagement::set_mode_on2(const std::shared_ptr<rmw_request_id_t> header)
{
  std::unique_lock<std::mutex> lock(mutex_, std::adopt_lock);
  mode_on_timer_->cancel();

  SetMode::Response res;
  if (operation_mode_.mode != OperationModeState::STOP) {
    res.status.code = ResponseStatus::ERROR;
    res.status.message = "operation mode is not stop";
  } else if (!store_.write(maintenance::State::ON)) {
    res.status.code = ResponseStatus::ERROR;
    res.status.message = "failed to write state";
  } else {
    res.status.code = ResponseStatus::SUCCESS;
  }

  is_maintenance_requesting_ = false;
  publish_diagnostics();
  srv_set_mode_->send_response(*header, res);
}

void MaintenanceManagement::on_timer()
{
  publish_diagnostics();
}

void MaintenanceManagement::publish_diagnostics()
{
  // clang-format off
  const auto stringify = [](const maintenance::State & state) {
    switch (state) {
      case maintenance::State::UNKNOWN: return "UNKNOWN";
      case maintenance::State::ON:      return "ON";
      case maintenance::State::OFF:     return "OFF";
      default:                          return "INVALID";
    }
  };
  // clang-format on

  const auto state = store_.read();
  const auto is_ok = (state == maintenance::State::OFF) && (!is_maintenance_requesting_);

  DiagnosticStatus status;
  status.name = std::string(this->get_name()) + ": state";
  status.level = is_ok ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
  status.message = stringify(state);

  DiagnosticArray msg;
  msg.header.stamp = this->now();
  msg.status.push_back(status);
  pub_diagnostics_->publish(msg);
}

}  // namespace tier4_maintenance_management

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_maintenance_management::MaintenanceManagement)
