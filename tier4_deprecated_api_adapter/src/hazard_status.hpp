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

#ifndef HAZARD_STATUS_HPP_
#define HAZARD_STATUS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>
#include <tier4_external_api_msgs/msg/hazard_status_stamped.hpp>

namespace tier4_deprecated_api_adapter
{

class HazardStatus : public rclcpp::Node
{
public:
  explicit HazardStatus(const rclcpp::NodeOptions & options);

private:
  using ExternalMessage = tier4_external_api_msgs::msg::HazardStatusStamped;
  using InternalMessage = autoware_system_msgs::msg::HazardStatusStamped;
  rclcpp::Publisher<ExternalMessage>::SharedPtr pub_;
  rclcpp::Subscription<InternalMessage>::SharedPtr sub_;
};

}  // namespace tier4_deprecated_api_adapter

#endif  // HAZARD_STATUS_HPP_
