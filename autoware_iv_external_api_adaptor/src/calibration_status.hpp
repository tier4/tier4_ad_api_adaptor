// Copyright 2021 TIER IV, Inc.
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

#ifndef CALIBRATION_STATUS_HPP_
#define CALIBRATION_STATUS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tier4_api_utils/tier4_api_utils.hpp"

#include <autoware/agnocast_wrapper/node.hpp>

#include "tier4_external_api_msgs/msg/calibration_status_array.hpp"
#include "tier4_external_api_msgs/srv/get_accel_brake_map_calibration_data.hpp"

namespace external_api
{

class CalibrationStatus : public autoware::agnocast_wrapper::Node
{
public:
  explicit CalibrationStatus(const rclcpp::NodeOptions & options);

private:
  using NodeT = autoware::agnocast_wrapper::Node;
  using GetAccelBrakeMapCalibrationData =
    tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData;
  using CalibrationStatusMsg = tier4_external_api_msgs::msg::CalibrationStatus;
  using CalibrationStatusArray = tier4_external_api_msgs::msg::CalibrationStatusArray;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  AUTOWARE_TIMER_PTR timer_;
  tier4_api_utils::Service<GetAccelBrakeMapCalibrationData, NodeT>::SharedPtr
    srv_get_accel_brake_map_calibration_data_;
  tier4_api_utils::Client<GetAccelBrakeMapCalibrationData, NodeT>::SharedPtr
    cli_get_accel_brake_map_calibration_data_;
  AUTOWARE_PUBLISHER_PTR(CalibrationStatusArray) pub_calibration_status_;
  AUTOWARE_SUBSCRIPTION_PTR(CalibrationStatusMsg) sub_accel_brake_map_calibration_status_;

  // ros callback
  void onTimer();
  void getAccelBrakeMapCalibrationData(
    const tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData::Response::SharedPtr
      response);

  // calibration status
  CalibrationStatusMsg::ConstSharedPtr accel_brake_map_status_;
};

}  // namespace external_api

#endif  // CALIBRATION_STATUS_HPP_
