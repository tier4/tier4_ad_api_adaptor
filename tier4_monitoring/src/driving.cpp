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

  pub_status_ = node.create_publisher<DrivingStatus>(
    "/monitoring/driving/status", rclcpp::QoS(1).transient_local());
  srv_enable_ = node.create_service<EnableDriving>(
    "/monitoring/driving/enable",
    std::bind(&Driving::on_enable, this, std::placeholders::_1, std::placeholders::_2));

  is_level2_available = false;
  is_level4_available = false;
}

void Driving::on_operation_mode(const OperationModeState & msg)
{
  operation_mode_ = msg;

  // is_level2_available = msg.is_autonomous_mode_available;
  // is_level4_available = msg.is_autonomous_mode_available;
}

void Driving::on_enable(
  const EnableDriving::Request::SharedPtr req, const EnableDriving::Response::SharedPtr res)
{
  (void)req;
  res->status.code = ResponseStatus::SUCCESS;
}

void Driving::publish(rclcpp::Time now)
{
  const auto get_mode = [this]() {
    if (operation_mode_.mode == OperationModeState::STOP) {
      return DrivingStatus::STOP;
    }
    if (operation_mode_.mode == OperationModeState::AUTONOMOUS) {
      if (request_level_ == DrivingLevel::kLevel2) return DrivingStatus::LEVEL2;
      if (request_level_ == DrivingLevel::kLevel4) return DrivingStatus::LEVEL4;
    }
    return DrivingStatus::UNKNOWN;
  };

  DrivingStatus msg;
  msg.stamp = now;
  msg.mode = get_mode();
  msg.is_level2_available = is_level2_available;
  msg.is_level4_available = is_level4_available;
  pub_status_->publish(msg);
}

}  // namespace tier4_monitoring
