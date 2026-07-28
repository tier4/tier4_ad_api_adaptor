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
#include <set>
#include <sstream>
#include <string>

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

  map_ = nullptr;
  is_level4_available_ = false;
}

void Lanelet::on_map(const LaneletMapBin & msg)
{
  map_ = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(msg);
  update_level4_availability();
}

void Lanelet::on_route(const Route & msg)
{
  route_ = msg;
  update_level4_availability();
}

void Lanelet::update_level4_availability()
{
  if (!map_) {
    RCLCPP_DEBUG(logger_, "vector map is not received yet");
    is_level4_available_ = false;
    return;
  }
  if (!route_ || route_->data.empty()) {
    RCLCPP_DEBUG(logger_, "route is not set");
    is_level4_available_ = false;
    return;
  }
  is_level4_available_ = check_level4_availability(route_.value());
}

bool Lanelet::check_level4_availability(const Route & route) const
{
  // Find the lanelet that has the tag specifying the end of the level4 section.
  const auto start_id = route.data.front().segments.front().preferred.id;
  if (!map_->laneletLayer.exists(start_id)) {
    RCLCPP_ERROR(logger_, "the start lanelet is not found in the map");
    return false;
  }

  auto start_lanelet = map_->laneletLayer.get(start_id);
  if (!start_lanelet.hasAttribute(level4_tag)) {
    // The start lanelet may be a road shoulder, so search the lanelets near the start pose.
    const auto candidates = get_tagged_lanelets(route.data.front().start);
    if (candidates.empty()) {
      RCLCPP_DEBUG(logger_, "the level4 tag is not found near the start pose");
      return false;
    }
    if (candidates.size() > 1) {
      RCLCPP_INFO(logger_, "there are multiple lanelets that have the level4 tag");
      return false;
    }
    start_lanelet = candidates.front();
  }

  // Check that the goal pose is on one of the lanelets specified by the tag.
  const auto goal_ids = parse_level4_tag(start_lanelet.attribute(level4_tag).value());
  if (!goal_ids) {
    RCLCPP_ERROR(logger_, "the level4 tag has an invalid value");
    return false;
  }

  for (const auto goal_id : *goal_ids) {
    if (!map_->laneletLayer.exists(goal_id)) {
      continue;
    }
    const auto goal_lanelet = map_->laneletLayer.get(goal_id);
    const auto & goal_pose = route.data.front().goal;
    if (autoware::experimental::lanelet2_utils::is_in_lanelet(goal_pose, goal_lanelet, 0.0)) {
      return true;
    }
  }

  RCLCPP_ERROR(logger_, "the goal pose is not on the lanelet specified by the level4 tag");
  return false;
}

lanelet::ConstLanelets Lanelet::get_tagged_lanelets(const geometry_msgs::msg::Pose & pose) const
{
  constexpr double search_radius = 5.0;  // meters
  const lanelet::BasicPoint2d point(pose.position.x, pose.position.y);

  lanelet::ConstLanelets result;
  const auto func = [&result, &point](
                      const lanelet::BoundingBox2d & box, const lanelet::ConstLanelet & lane) {
    if (
      lane.hasAttribute(level4_tag) &&
      boost::geometry::covered_by(point, lane.polygon2d().basicPolygon())) {
      result.push_back(lane);
    }
    if (result.size() > 1) return true;  // Having multiple candidates is an error.
    return search_radius < boost::geometry::distance(box, point);
  };
  map_->laneletLayer.nearestUntil(point, func);
  return result;
}

std::optional<std::set<int64_t>> Lanelet::parse_level4_tag(const std::string & text)
{
  std::set<int64_t> result;
  std::istringstream stream(text);
  std::string token;

  while (std::getline(stream, token, ',')) {
    if (token.empty()) {
      return std::nullopt;
    }
    for (const auto c : token) {
      if (!std::isdigit(c)) return std::nullopt;
    }
    try {
      result.insert(std::stoll(token));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return result;
}

}  // namespace tier4_monitoring
