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

#include "manual_mode.hpp"

namespace tier4_deprecated_api_adapter
{

ManualMode::ManualMode(const rclcpp::NodeOptions & options) : Node("manual_mode", options)
{
  const auto adaptor = apiutils::NodeAdaptor(this);
  adaptor.init_srv(srv_set_operator_, this, &ManualMode::on_set_operator);
  adaptor.init_pub(pub_operator_);
  adaptor.init_pub(pub_observer_);
  adaptor.init_sub(sub_operation_mode_, this, &ManualMode::on_operation_mode);

  const auto period = rclcpp::Rate(5.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { this->on_timer(); });
}

void ManualMode::on_timer()
{
}

void ManualMode::on_operation_mode(const OperationModeState::Message & msg)
{
  (void)msg;
}

void ManualMode::on_set_operator(
  const SetOperator::Service::Request::SharedPtr req,
  const SetOperator::Service::Response::SharedPtr res)
{
  (void)req;
  (void)res;
}

}  // namespace tier4_deprecated_api_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_deprecated_api_adapter::ManualMode)
