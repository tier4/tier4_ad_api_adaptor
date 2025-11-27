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

#include "hazard_status.hpp"

#include <unordered_set>
#include <utility>
#include <vector>

namespace tier4_deprecated_api_adapter
{

HazardStatus::HazardStatus(const rclcpp::NodeOptions & options) : Node("hazard_status", options)
{
  const auto on_message = [this](const InternalMessage & internal) {
    ExternalMessage external;
    external.stamp = internal.stamp;
    pub_->publish(external);
  };

  pub_ = create_publisher<ExternalMessage>("~/hazard_status", rclcpp::QoS(1));
  sub_ = create_subscription<InternalMessage>("/system/emergency/hazard_status", 1, on_message);
}

}  // namespace tier4_deprecated_api_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_deprecated_api_adapter::HazardStatus)
