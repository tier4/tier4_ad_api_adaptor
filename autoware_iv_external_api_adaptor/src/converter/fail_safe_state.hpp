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

#ifndef CONVERTER__FAIL_SAFE_STATE_HPP_
#define CONVERTER__FAIL_SAFE_STATE_HPP_

#include "autoware_external_api_msgs/msg/fail_safe_state.hpp"
#include "autoware_external_api_msgs/msg/fail_safe_state_stamped.hpp"
#include "autoware_system_msgs/msg/emergency_state.hpp"
#include "autoware_system_msgs/msg/emergency_state_stamped.hpp"

namespace external_api::converter
{

using ExternalFailSafeState = autoware_external_api_msgs::msg::FailSafeState;
using InternalFailSafeState = autoware_system_msgs::msg::EmergencyState;
using ExternalFailSafeStateStamped = autoware_external_api_msgs::msg::FailSafeStateStamped;
using InternalFailSafeStateStamped = autoware_system_msgs::msg::EmergencyStateStamped;

ExternalFailSafeState to_external(const InternalFailSafeState & msg)
{
  using External = ExternalFailSafeState;
  using Internal = InternalFailSafeState;

  auto builder = autoware_external_api_msgs::build<External>();
  switch (msg.state) {
    case Internal::NORMAL:
      return builder.state(External::NORMAL);
    case Internal::OVERRIDE_REQUESTING:
      return builder.state(External::OVERRIDE_REQUESTING);
    case Internal::MRM_OPERATING:
      return builder.state(External::MRM_OPERATING);
    case Internal::MRM_SUCCEEDED:
      return builder.state(External::MRM_SUCCEEDED);
    case Internal::MRM_FAILED:
      return builder.state(External::MRM_FAILED);
  }
  throw std::out_of_range("fail_safe_state=" + std::to_string(msg.state));
}

ExternalFailSafeStateStamped to_external(const InternalFailSafeStateStamped & msg)
{
  return autoware_external_api_msgs::build<ExternalFailSafeStateStamped>()
         .stamp(msg.stamp).state(to_external(msg.state));
}

}  // namespace external_api::converter

#endif  // CONVERTER__FAIL_SAFE_STATE_HPP_
