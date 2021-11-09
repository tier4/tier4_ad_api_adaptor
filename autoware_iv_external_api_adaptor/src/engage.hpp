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

#ifndef ENGAGE_HPP_
#define ENGAGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_api_mapping/external/set/engage.hpp"
#include "autoware_api_mapping/external/get/engage.hpp"
#include "autoware_api_mapping/internal/set/engage.hpp"
#include "autoware_api_mapping/internal/get/engage.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"

namespace external_api
{

class Engage : public rclcpp::Node
{
public:
  explicit Engage(const rclcpp::NodeOptions & options);

private:
  using ExternalEngageAPI = autoware_api_mapping::external::set::engage::API;
  using ExternalStatusAPI = autoware_api_mapping::external::get::engage::API;
  using InternalEngageAPI = autoware_api_mapping::internal::set::engage::API;
  using InternalStatusAPI = autoware_api_mapping::internal::get::engage::API;
  using AutowareState = autoware_system_msgs::msg::AutowareState;

  // ros interface
  rclcpp::CallbackGroup::SharedPtr group_;
  ExternalEngageAPI::Service srv_set_engage_;
  InternalEngageAPI::Client cli_set_engage_;
  ExternalStatusAPI::Publisher pub_get_engage_;
  InternalStatusAPI::Subscription sub_get_engage_;
  rclcpp::Subscription<AutowareState>::SharedPtr sub_autoware_state_;

  // class state
  bool waiting_for_engage_;

  // ros callback
  void setEngage(
    const ExternalEngageAPI::RequestType::SharedPtr request,
    const ExternalEngageAPI::ResponseType::SharedPtr response);
  void onEngageStatus(
    const InternalStatusAPI::MessageType::SharedPtr message);
  void onAutowareState(
    const autoware_system_msgs::msg::AutowareState::SharedPtr message);
};

}  // namespace external_api

#endif  // ENGAGE_HPP_
