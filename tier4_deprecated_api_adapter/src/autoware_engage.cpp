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

#include "autoware_engage.hpp"

namespace tier4_deprecated_api_adapter
{

AutowareEngage::AutowareEngage(const rclcpp::NodeOptions & options)
: Node("autoware_engage", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  auto_operator_change_ = declare_parameter("auto_operator_change", false);
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pub_engage_ = create_publisher<EngageStatus>("/api/external/get/engage", rclcpp::QoS(1));
  srv_engage_ = create_service<EngageService>(
    "/api/external/set/engage", std::bind(&AutowareEngage::on_engage, this, _1, _2));

  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    std::bind(&AutowareEngage::on_state, this, _1));
  cli_change_stop_mode_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_stop", rmw_qos_profile_services_default, callback_group_);
  cli_change_autonomous_mode_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous", rmw_qos_profile_services_default, callback_group_);
  cli_change_autoware_control_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/enable_autoware_control", rmw_qos_profile_services_default,
    callback_group_);

  state_.mode = OperationModeState::UNKNOWN;
}

void AutowareEngage::on_state(const OperationModeState & msg)
{
  (void)msg;
}

void AutowareEngage::on_engage(
  const EngageService::Request::SharedPtr req, EngageService::Response::SharedPtr res)
{
  (void)req;
  (void)res;
}

}  // namespace tier4_deprecated_api_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_deprecated_api_adapter::AutowareEngage)
