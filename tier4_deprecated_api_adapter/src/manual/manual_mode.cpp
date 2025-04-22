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

#include <tier4_api_utils/types/response.hpp>

#include <memory>

namespace tier4_deprecated_api_adapter
{

ManualMode::ManualMode(const rclcpp::NodeOptions & options) : Node("manual_mode", options)
{
  const auto adaptor = apiutils::NodeAdaptor(this);
  adaptor.init_srv(srv_set_operator_, this, &ManualMode::on_set_operator);
  adaptor.init_cli(cli_autonomous_mode_);
  adaptor.init_cli(cli_autoware_control_);
  adaptor.init_cli(cli_platform_control_);

  adaptor.init_pub(pub_operator_);
  adaptor.init_pub(pub_observer_);
  adaptor.init_sub(sub_operation_mode_, this, &ManualMode::on_operation_mode);

  const auto period = rclcpp::Rate(5.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { this->on_timer(); });
}

void ManualMode::on_timer()
{
  if (current_operator_) {
    GetOperator::Message msg;
    msg.mode = *current_operator_;
    pub_operator_->publish(msg);
  }
  if (current_observer_) {
    GetObserver::Message msg;
    msg.mode = *current_observer_;
    pub_observer_->publish(msg);
  }
}

void ManualMode::on_operation_mode(const OperationModeState::Message & msg)
{
  const auto get_operator = [](const OperationModeState::Message & msg) -> std::optional<uint8_t> {
    if (!msg.is_autoware_control_enabled) {
      return GetOperator::Message::DRIVER;
    }
    switch (msg.mode) {
      case OperationModeState::Message::STOP:
      case OperationModeState::Message::AUTONOMOUS:
        return GetOperator::Message::AUTONOMOUS;
      case OperationModeState::Message::LOCAL:
      case OperationModeState::Message::REMOTE:
        return GetOperator::Message::OBSERVER;
      default:
        return std::nullopt;
    }
  };

  const auto get_observer = [](const OperationModeState::Message & msg) -> std::optional<uint8_t> {
    switch (msg.mode) {
      case OperationModeState::Message::STOP:
      case OperationModeState::Message::AUTONOMOUS:
        return GetObserver::Message::REMOTE;  // Use remote as default.
      case OperationModeState::Message::REMOTE:
        return GetObserver::Message::REMOTE;
      case OperationModeState::Message::LOCAL:
        return GetObserver::Message::LOCAL;
      default:
        return std::nullopt;
    }
  };

  const auto new_operator = get_operator(msg);
  if (new_operator) {
    current_operator_ = *new_operator;
  }

  const auto new_observer = get_observer(msg);
  if (new_observer) {
    current_observer_ = *new_observer;
  }
}

void ManualMode::on_set_operator(
  const SetOperator::Service::Request::SharedPtr req,
  const SetOperator::Service::Response::SharedPtr res)
{
  if (req->mode.mode == GetOperator::Message::DRIVER) {
    const auto api_req = std::make_shared<DisableAutowareControl::Service::Request>();
    const auto api_res = cli_platform_control_->call(api_req);
    res->status = tier4_api_utils::response_from(api_res->status);
    return;
  }

  (void)req;
  (void)res;
}

}  // namespace tier4_deprecated_api_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_deprecated_api_adapter::ManualMode)
