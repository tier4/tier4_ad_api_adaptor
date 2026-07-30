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

#include "driving.hpp"

#include "message.hpp"

namespace tier4_monitoring
{

Driving::Driving(rclcpp::Node & node)
{
  sub_operation_mode_ = node.create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    std::bind(&Driving::on_operation_mode, this, std::placeholders::_1));
  cli_change_stop_mode =
    node.create_client<ChangeOperationMode>("/api/operation_mode/change_to_stop");
  cli_change_autonomous_mode =
    node.create_client<ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

  pub_status_ = node.create_publisher<DrivingStatus>(
    "/api/external/get/monitoring/driving/status", rclcpp::QoS(1).transient_local());
  srv_enable_ = node.create_service<EnableDriving>(
    "/api/external/set/monitoring/driving/enable",
    std::bind(&Driving::on_enable, this, std::placeholders::_1, std::placeholders::_2));

  pub_velocity_limit_set_ = node.create_publisher<VelocityLimitSet>(
    "/planning/scenario_planning/max_velocity_candidates", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_clear_ = node.create_publisher<VelocityLimitClear>(
    "/planning/scenario_planning/clear_velocity_limit", rclcpp::QoS{1}.transient_local());

  current_level_ = DrivingLevel::kUnknown;
  is_level2_available = false;
  is_level4_available = false;
  velocity_limit_requested_ = false;
}

void Driving::update_available_levels(bool level2, bool level4)
{
  is_level2_available = level2;
  is_level4_available = level4;
}

void Driving::on_operation_mode(const OperationModeState & msg)
{
  operation_mode_ = msg;
}

void Driving::on_enable(
  const EnableDriving::Request::SharedPtr req, const EnableDriving::Response::SharedPtr res)
{
  // Handle stop and unknown requests.
  const auto level = from_driving_status(req->mode);
  if (level == DrivingLevel::kUnknown) {
    res->status.code = ResponseStatus::ERROR;
    res->status.message = "unknown mode";
    return;
  }
  if (level == DrivingLevel::kStop) {
    cli_change_stop_mode->async_send_request(std::make_shared<ChangeOperationMode::Request>());
    res->status.code = ResponseStatus::SUCCESS;
    return;
  }

  // Handle level2 and level4 requests.
  if (!operation_mode_.is_autonomous_mode_available) {
    res->status.code = ResponseStatus::ERROR;
    res->status.message = "autonomous mode is not available";
    return;
  }
  if (level == DrivingLevel::kLevel2 && !is_level2_available) {
    res->status.code = ResponseStatus::ERROR;
    res->status.message = "level2 is not available";
    return;
  }
  if (level == DrivingLevel::kLevel4 && !is_level4_available) {
    res->status.code = ResponseStatus::ERROR;
    res->status.message = "level4 is not available";
    return;
  }
  current_level_ = level;
  cli_change_autonomous_mode->async_send_request(std::make_shared<ChangeOperationMode::Request>());
  res->status.code = ResponseStatus::SUCCESS;
}

void Driving::update(const rclcpp::Time & now)
{
  bool error = false;
  if (current_level_ == DrivingLevel::kLevel2 && !is_level2_available) error = true;
  if (current_level_ == DrivingLevel::kLevel4 && !is_level4_available) error = true;

  if (error) {
    set_velocity_limit(now);
  } else {
    clear_velocity_limit(now);
  }
}

void Driving::publish(const rclcpp::Time & now)
{
  const auto get_level = [this]() {
    if (operation_mode_.mode == OperationModeState::STOP) {
      return DrivingLevel::kStop;
    }
    if (operation_mode_.mode == OperationModeState::AUTONOMOUS) {
      if (current_level_ == DrivingLevel::kLevel2) return DrivingLevel::kLevel2;
      if (current_level_ == DrivingLevel::kLevel4) return DrivingLevel::kLevel4;
    }
    return DrivingLevel::kUnknown;
  };

  DrivingStatus msg;
  msg.mode = to_driving_status(get_level());
  msg.is_level2_available = is_level2_available && operation_mode_.is_autonomous_mode_available;
  msg.is_level4_available = is_level4_available && operation_mode_.is_autonomous_mode_available;
  if (prev_status_ != msg) {
    prev_status_ = msg;
    msg.stamp = now;
    pub_status_->publish(msg);
  }
}

void Driving::set_velocity_limit(const rclcpp::Time & now)
{
  if (velocity_limit_requested_) return;
  velocity_limit_requested_ = true;

  VelocityLimitSet msg;
  msg.stamp = now;
  msg.max_velocity = 0;
  msg.use_constraints = false;
  msg.sender = "monitoring_api";
  pub_velocity_limit_set_->publish(msg);
}

void Driving::clear_velocity_limit(const rclcpp::Time & now)
{
  if (!velocity_limit_requested_) return;
  velocity_limit_requested_ = false;

  VelocityLimitClear msg;
  msg.stamp = now;
  msg.command = true;
  msg.sender = "monitoring_api";
  pub_velocity_limit_clear_->publish(msg);
}

}  // namespace tier4_monitoring
