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

#include "traffic_light.hpp"

namespace tier4_autoware_api_extension
{

auto convert(const autoware_perception_msgs::msg::TrafficLightElement & internal)
{
  tier4_external_api_msgs::msg::TrafficLightElement external;
  external.color = internal.color;
  external.shape = internal.shape;
  external.status = internal.status;
  external.confidence = internal.confidence;
  return external;
}

TrafficLight::TrafficLight(const rclcpp::NodeOptions & options) : Node("traffic_light", options)
{
  sub_traffic_light_group_ = create_subscription<InternalMessage>(
    "/planning/scenario_planning/lane_driving/behavior_planning/debug/traffic_signal", 1,
    std::bind(&TrafficLight::on_message, this, std::placeholders::_1));

  pub_traffic_light_group_ =
    create_publisher<ExternalMessage>("/api/external/get/nearest_traffic_light_group", 1);
}

void TrafficLight::on_message(const InternalMessage & internal)
{
  ExternalMessage external;
  external.traffic_light_group_id = internal.traffic_light_group_id;
  for (const auto & element : internal.elements) {
    external.elements.push_back(convert(element));
  }
  pub_traffic_light_group_->publish(external);
}

}  // namespace tier4_autoware_api_extension

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_autoware_api_extension::TrafficLight)
