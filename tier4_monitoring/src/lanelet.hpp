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
#include <set>
#include <string>

namespace tier4_monitoring
{

class Lanelet
{
public:
  explicit Lanelet(rclcpp::Node & node);
  bool is_level4_available() const { return is_level4_available_; }

private:
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using Route = autoware_adapi_v1_msgs::msg::Route;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Route>::SharedPtr sub_route_;

  void on_map(const LaneletMapBin & msg);
  void on_route(const Route & msg);

  void update_level4_availability();
  bool check_level4_availability(const Route & route) const;
  lanelet::ConstLanelets get_tagged_lanelets(const geometry_msgs::msg::Pose & pose) const;
  static std::optional<std::set<int64_t>> parse_level4_tag(const std::string & text);

  rclcpp::Logger logger_;
  lanelet::LaneletMapConstPtr map_;
  std::optional<Route> route_;
  bool is_level4_available_;

  static constexpr char level4_tag[] = "level4_operation_end_lanelet";
};

}  // namespace tier4_monitoring

#endif  // LANELET_HPP_
