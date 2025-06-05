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

#include "route_distance.hpp"

namespace tier4_autoware_api_extension
{

RouteDistance::RouteDistance(const rclcpp::NodeOptions & options) : Node("route_distance", options)
{
  sub_route_distance_ = create_subscription<InternalMessage>(
    "/tier4_api/utils/path_distance_calculator/distance", 1,
    std::bind(&RouteDistance::on_message, this, std::placeholders::_1));

  pub_route_distance_ = create_publisher<ExternalMessage>("/api/external/get/route_distance", 1);
}

void RouteDistance::on_message(const InternalMessage & internal)
{
  ExternalMessage external;
  external.stamp = internal.stamp;
  external.remaining_distance = internal.data;
  pub_route_distance_->publish(external);
}

}  // namespace tier4_autoware_api_extension

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_autoware_api_extension::RouteDistance)
