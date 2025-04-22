// Copyright 2025 TIER IV, Inc.
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

#ifndef MANUAL__MANUAL_MODE_HPP_
#define MANUAL__MANUAL_MODE_HPP_

#include <autoware/adapi_specs/operation_mode.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/observer.hpp>
#include <tier4_external_api_msgs/msg/operator.hpp>
#include <tier4_external_api_msgs/srv/set_operator.hpp>

#include <optional>

namespace tier4_deprecated_api_adapter
{

namespace apiutils = autoware::component_interface_utils;
using autoware::adapi_specs::operation_mode::ChangeToAutonomous;
using autoware::adapi_specs::operation_mode::DisableAutowareControl;
using autoware::adapi_specs::operation_mode::EnableAutowareControl;
using autoware::adapi_specs::operation_mode::OperationModeState;
using tier4_external_api_msgs::msg::Observer;
using tier4_external_api_msgs::msg::Operator;

struct SetOperator
{
  using Service = tier4_external_api_msgs::srv::SetOperator;
  static constexpr char name[] = "/api/external/set/operator";
};

struct GetOperator
{
  using Message = tier4_external_api_msgs::msg::Operator;
  static constexpr char name[] = "/api/external/get/operator";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct GetObserver
{
  using Message = tier4_external_api_msgs::msg::Observer;
  static constexpr char name[] = "/api/external/get/observer";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

class ManualMode : public rclcpp::Node
{
public:
  explicit ManualMode(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_;

  apiutils::Service<SetOperator>::SharedPtr srv_set_operator_;
  apiutils::Publisher<GetOperator>::SharedPtr pub_operator_;
  apiutils::Publisher<GetObserver>::SharedPtr pub_observer_;

  apiutils::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  apiutils::Client<ChangeToAutonomous>::SharedPtr cli_autonomous_mode_;
  apiutils::Client<EnableAutowareControl>::SharedPtr cli_autoware_control_;
  apiutils::Client<DisableAutowareControl>::SharedPtr cli_platform_control_;

  void on_timer();
  void on_operation_mode(const OperationModeState::Message & msg);
  void on_set_operator(
    const SetOperator::Service::Request::SharedPtr req,
    const SetOperator::Service::Response::SharedPtr res);

  std::optional<uint8_t> current_operator_;
  std::optional<uint8_t> current_observer_;
};

}  // namespace tier4_deprecated_api_adapter

#endif  // MANUAL__MANUAL_MODE_HPP_
