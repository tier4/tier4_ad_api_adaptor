// Copyright 2026 TIER IV, Inc.
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

#ifndef LANELET_HPP_
#define LANELET_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <optional>
#include <string>
#include <unordered_set>

namespace tier4_monitoring
{

class Lanelet
{
public:
  explicit Lanelet(rclcpp::Node & node);
  bool is_level2_available() const { return true; }
  bool is_level4_available() const { return is_level4_available_; }

private:
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using Route = autoware_adapi_v1_msgs::msg::Route;
  using RouteData = autoware_adapi_v1_msgs::msg::RouteData;

  rclcpp::Logger logger_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Route>::SharedPtr sub_route_;

  void on_map(const LaneletMapBin & msg);
  void on_route(const Route & msg);

  void update_level4_availability();
  bool check_level4_availability() const;

  static lanelet::ConstLanelets get_lanelets_with_adjacent_road_shoulder(
    const lanelet::LaneletMapConstPtr & map, const lanelet::Id & id);
  static std::optional<std::unordered_set<lanelet::Id>> get_goal_ids_from_level4_tag(
    const lanelet::ConstLanelet & lanelet);

  bool is_level4_available_;
  lanelet::LaneletMapConstPtr map_;
  std::optional<RouteData> route_;
};

}  // namespace tier4_monitoring

#endif  // LANELET_HPP_
