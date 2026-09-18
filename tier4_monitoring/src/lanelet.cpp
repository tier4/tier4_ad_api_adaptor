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

#include "lanelet.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>

namespace tier4_monitoring
{

Lanelet::Lanelet(rclcpp::Node & node) : logger_(node.get_logger())
{
  using std::placeholders::_1;

  sub_map_ = node.create_subscription<LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&Lanelet::on_map, this, _1));
  sub_route_ = node.create_subscription<Route>(
    "/api/routing/route", rclcpp::QoS(1).transient_local(),
    std::bind(&Lanelet::on_route, this, _1));

  is_level4_available_ = false;
  map_ = nullptr;
  route_ = std::nullopt;
}

void Lanelet::on_map(const LaneletMapBin & msg)
{
  map_ = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(msg);
  update_level4_availability();
}

void Lanelet::on_route(const Route & msg)
{
  if (msg.data.empty()) {
    route_ = std::nullopt;
  } else {
    route_ = msg.data.front();
  }
  update_level4_availability();
}

void Lanelet::update_level4_availability()
{
  if (!map_) {
    is_level4_available_ = false;
    return;
  }
  if (!route_) {
    is_level4_available_ = false;
    return;
  }
  is_level4_available_ = check_level4_availability();
}

bool Lanelet::check_level4_availability() const
{
  const auto & route = route_.value();
  if (route.segments.empty()) {
    RCLCPP_ERROR(logger_, "the route has no segments");
    return false;
  }

  const auto start_id = route.segments.front().preferred.id;
  const auto goal_id = route.segments.back().preferred.id;
  std::unordered_set<lanelet::Id> level4_goal_candidates;

  const auto start_lanelets = get_lanelets_with_adjacent_road_shoulder(map_, start_id);
  for (const auto & lanelet : start_lanelets) {
    const auto ids = get_goal_ids_from_level4_tag(lanelet);
    if (!ids) {
      RCLCPP_ERROR_STREAM(logger_, "invalid level4 attr value in lanelet: " << lanelet.id());
      return false;
    }
    for (const auto & id : ids.value()) {
      level4_goal_candidates.insert(id);
    }
  }

  const auto goal_lanelets = get_lanelets_with_adjacent_road_shoulder(map_, goal_id);
  for (const auto & lanelet : goal_lanelets) {
    if (level4_goal_candidates.count(lanelet.id())) {
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets Lanelet::get_lanelets_with_adjacent_road_shoulder(
  const lanelet::LaneletMapConstPtr & map, const lanelet::Id & id)
{
  lanelet::ConstLanelets result;
  if (!map->laneletLayer.exists(id)) {
    return result;
  }

  const auto target = map->laneletLayer.get(id);
  result.push_back(target);

  const auto bounds = {target.leftBound(), target.rightBound()};
  for (const auto & bound : bounds) {
    lanelet::ConstLanelets lanelets = map->laneletLayer.findUsages(bound);
    for (const auto & lanelet : lanelets) {
      const std::string subtype = lanelet.attributeOr(lanelet::AttributeNamesString::Subtype, "");
      if (subtype == "road_shoulder") {
        result.push_back(lanelet);
      }
    }
  }
  return result;
}

std::optional<std::unordered_set<lanelet::Id>> Lanelet::get_goal_ids_from_level4_tag(
  const lanelet::ConstLanelet & lanelet)
{
  const auto strict_stoll = [](const std::string & str) -> std::optional<lanelet::Id> {
    for (const auto & c : str) {
      if (!std::isdigit(c)) return std::nullopt;
    }
    try {
      return std::stoll(str);
    } catch (const std::exception &) {
      return std::nullopt;
    }
  };

  static constexpr char level4_tag[] = "level4_operation_end_lanelet";

  std::unordered_set<lanelet::Id> result;
  if (!lanelet.hasAttribute(level4_tag)) {
    return result;  // It is not an error if the tag is not found.
  }
  std::istringstream stream(lanelet.attribute(level4_tag).value());
  std::string token;
  while (std::getline(stream, token, ',')) {
    const auto id = strict_stoll(token);
    if (!id) return std::nullopt;
    result.insert(id.value());
  }
  return result;
}

}  // namespace tier4_monitoring
