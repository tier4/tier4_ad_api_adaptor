// Copyright 2021 Tier IV, Inc.
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

#include "engage.hpp"
#include <memory>

namespace external_api
{

Engage::Engage(const rclcpp::NodeOptions & options)
: Node("external_api_engage", options)
{
  using namespace std::placeholders;

  autoware_api_utils::ServiceProxyNodeInterface proxy(this);
  auto srv_callback = std::bind(&Engage::setEngage, this, _1, _2);
  auto sub_callback = std::bind(&Engage::onEngageStatus, this, _1);

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_engage_ = ExternalEngageAPI::create_service(&proxy, srv_callback, group_);
  cli_set_engage_ = InternalEngageAPI::create_client(&proxy);
  pub_get_engage_ = ExternalStatusAPI::create_publisher(this);
  sub_get_engage_ = InternalStatusAPI::create_subscription(this, sub_callback);

  sub_autoware_state_ = create_subscription<autoware_system_msgs::msg::AutowareState>(
    "/autoware/state", rclcpp::QoS(1),
    std::bind(&Engage::onAutowareState, this, _1));

  waiting_for_engage_ = false;
}

void Engage::setEngage(
  const ExternalEngageAPI::RequestType::SharedPtr request,
  const ExternalEngageAPI::ResponseType::SharedPtr response)
{
  if (request->engage && !waiting_for_engage_) {
    response->status = autoware_api_utils::response_error("It is not ready to engage.");
    return;
  }

  auto [status, resp] = cli_set_engage_->call(request);
  if (!autoware_api_utils::is_success(status)) {
    response->status = status;
    return;
  }
  response->status = resp->status;
}

void Engage::onEngageStatus(
  const InternalStatusAPI::MessageType::SharedPtr message)
{
  pub_get_engage_->publish(
    ExternalStatusAPI::build_message().stamp(message->stamp).engage(message->engage));
}

void Engage::onAutowareState(
  const autoware_system_msgs::msg::AutowareState::SharedPtr message)
{
  using autoware_system_msgs::msg::AutowareState;
  waiting_for_engage_ = (message->state == AutowareState::WAITING_FOR_ENGAGE);
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::Engage)
