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

#include "lanelet.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>

namespace autoware::level4_mode_manager
{
Level4ModeManager::Level4ModeManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("level4_mode_manager", options), lanelet_map_ptr_(nullptr)
{
  is_initialized_ = false;
  is_level4_available_ = false;
  is_level4_driving_ = false;

  awaiting_response_ = false;
  awaiting_autonomous_ = false;
  latest_enable_level4_driving_ = false;
  response_waiting_counter_ = 0;

  lanelet_map_bin_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&Level4ModeManager::lanelet_map_bin_callback, this, std::placeholders::_1));
  operation_mode_state_sub_ = create_subscription<OperationModeState>(
    "input/operation_mode_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&Level4ModeManager::operation_mode_state_callback, this, std::placeholders::_1));
  route_sub_ = create_subscription<Route>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&Level4ModeManager::route_callback, this, std::placeholders::_1));

  level4_availability_pub_ = create_publisher<Level4Availability>(
    "output/is_level4_available", rclcpp::QoS{1}.transient_local());
  level4_driving_status_pub_ = create_publisher<Level4DrivingStatus>(
    "output/is_level4_driving", rclcpp::QoS{1}.transient_local());

  enable_level4_driving_srv_ = this->create_service<EnableLevel4Driving>(
    "service/enable_level4_driving", std::bind(
                                       &Level4ModeManager::call_select_level_and_run, this,
                                       std::placeholders::_1, std::placeholders::_2));
  override_level_srv_ = this->create_service<OverrideLevel>(
    "service/override_level",
    std::bind(
      &Level4ModeManager::call_override_level, this, std::placeholders::_1, std::placeholders::_2));

  change_operation_mode_cli_ =
    this->create_client<ChangeOperationMode>("service/change_to_autonomous");

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(1000),
    std::bind(&Level4ModeManager::on_timer, this));

  const rclcpp::Time current_time = this->now();

  Level4Availability availability;
  availability.stamp = current_time;
  availability.is_level4_available = is_level4_available_;

  Level4DrivingStatus driving_status;
  driving_status.stamp = current_time;
  driving_status.is_level4_driving = is_level4_driving_;

  level4_availability_pub_->publish(availability);
  level4_driving_status_pub_->publish(driving_status);
}

void Level4ModeManager::lanelet_map_bin_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_map_msg)
{
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*input_map_msg);
  RCLCPP_DEBUG(this->get_logger(), "Vector map updated!!");
}

void Level4ModeManager::operation_mode_state_callback(
  const OperationModeState::ConstSharedPtr input_operation_mode_state_msg)
{
  operation_mode_state_ = *input_operation_mode_state_msg;

  if (operation_mode_state_->mode == OperationModeState::AUTONOMOUS && awaiting_autonomous_) {
    awaiting_autonomous_ = false;
    if (is_level4_driving_ != latest_enable_level4_driving_) {
      is_level4_driving_ = latest_enable_level4_driving_;
      Level4DrivingStatus driving_status;
      driving_status.stamp = this->now();
      driving_status.is_level4_driving = is_level4_driving_;
      level4_driving_status_pub_->publish(driving_status);
    }
  }
}

void Level4ModeManager::route_callback(const Route::ConstSharedPtr input_route_msg)
{
  route_ = *input_route_msg;

  if (operation_mode_state_->mode == OperationModeState::AUTONOMOUS) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Route has changed during autonomous driving. Assuming subtle adjustment of planning.");
    return;
  }

  if (route_.value().data.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Route is empty! Ignore and keep availability.");
    return;
  }

  if (check_level4_availability(route_.value()) != is_level4_available_) {
    is_level4_available_ = !is_level4_available_;
    Level4Availability availability;
    availability.stamp = this->now();
    availability.is_level4_available = is_level4_available_;
    level4_availability_pub_->publish(availability);
  }
}

void Level4ModeManager::call_select_level_and_run(
  const EnableLevel4Driving::Request::SharedPtr request,
  EnableLevel4Driving::Response::SharedPtr response)
{
  if (!is_initialized_) {
    response->status.code = EnableLevel4Driving::Response::ERROR_NOT_INITIALIZED;
    response->status.message = "Not initialized yet.";
    return;
  }

  if (route_.value().data.empty()) {
    response->status.code = EnableLevel4Driving::Response::ERROR_CANNOT_START_AUTOWARE;
    response->status.message = "Service called when the route is empty.";
    return;
  }

  if (!operation_mode_state_->is_autonomous_mode_available) {
    response->status.code = EnableLevel4Driving::Response::ERROR_CANNOT_START_AUTOWARE;
    response->status.message = "AUTONOMOUS mode not available now.";
    return;
  }

  if (operation_mode_state_->mode == OperationModeState::AUTONOMOUS) {
    response->status.code = EnableLevel4Driving::Response::ERROR_CANNOT_START_AUTOWARE;
    response->status.message = "Autoware is AUTONOMOUS mode already.";
    return;
  }

  if (request->enable_level4_driving && !is_level4_available_) {
    response->status.code = EnableLevel4Driving::Response::ERROR_INVALID_LEVEL;
    response->status.message = "Level 4 driving is not available.";
    return;
  }

  if (!change_operation_mode_cli_->wait_for_service(std::chrono::milliseconds(3000))) {
    response->status.code = EnableLevel4Driving::Response::ERROR_CANNOT_START_AUTOWARE;
    response->status.message = "Cannot find the service to make Autoware AUTONOMOUS.";
    return;
  }

  latest_enable_level4_driving_ = request->enable_level4_driving;

  auto request_to_autoware = std::make_shared<ChangeOperationMode::Request>();
  awaiting_response_ = true;
  awaiting_autonomous_ = true;
  response_waiting_counter_ = 0;
  auto future = change_operation_mode_cli_->async_send_request(
    request_to_autoware, [this](rclcpp::Client<ChangeOperationMode>::SharedFuture response) {
      awaiting_response_ = false;
      if (response.get()->status.success) {
        RCLCPP_DEBUG(this->get_logger(), "Changed Autoware to AUTONOMOUS.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Couldn't changed Autoware to AUTONOMOUS!!");
      }
    });

  response->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
  response->status.message = "Sent request to Autoware.";
}

void Level4ModeManager::call_override_level(
  const OverrideLevel::Request::SharedPtr request, OverrideLevel::Response::SharedPtr response)
{
  if (!is_initialized_) {
    response->status.code = OverrideLevel::Response::ERROR_NOT_INITIALIZED;
    response->status.message = "Not initialized yet.";
    return;
  }

  if (motion_state_sub_.take_data()->state != MotionState::STOPPED) {
    response->status.code = OverrideLevel::Response::ERROR_VEHICLE_NOT_STOPPED;
    response->status.message = "Cannot override level when the vehicle is running.";
    return;
  }

  if (request->target_level == OverrideLevel::Request::LEVEL2) {
    if (is_level4_driving_) {
      is_level4_driving_ = false;
      Level4DrivingStatus driving_status;
      driving_status.stamp = this->now();
      driving_status.is_level4_driving = is_level4_driving_;
      level4_driving_status_pub_->publish(driving_status);
      response->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
      response->status.message = "Overrode driving level to Level 2.";
    } else {
      response->status.code = OverrideLevel::Response::ERROR_INVALID_TARGET_LEVEL;
      response->status.message = "Current driving level is already Level 2.";
    }
    return;
  }

  if (request->target_level == OverrideLevel::Request::LEVEL4) {
    response->status.code = tier4_external_api_msgs::msg::ResponseStatus::ERROR;
    response->status.message =
      "Currently there is no feature to force update the driving level to Level 4.";
    return;
  }

  response->status.code = OverrideLevel::Response::ERROR_INVALID_TARGET_LEVEL;
  response->status.message = "Invalid target level has been input!!";
}

void Level4ModeManager::on_timer()
{
  if (!lanelet_map_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 60000, "Vector map not found yet!!");
    return;
  }

  if (!operation_mode_state_) {
    RCLCPP_DEBUG(this->get_logger(), "Operation mode state not arrived yet!!");
    return;
  }

  if (!is_initialized_) {
    RCLCPP_INFO(this->get_logger(), "level4_mode_manager finished initialization.");
    is_initialized_ = true;

    if (route_ && !route_.value().data.empty()) {
      RCLCPP_INFO(
        this->get_logger(), "A valid route had already come. Check Level 4 availability.");
      is_level4_available_ = check_level4_availability(route_.value());
      Level4Availability availability;
      availability.stamp = this->now();
      availability.is_level4_available = is_level4_available_;
      level4_availability_pub_->publish(availability);
    }
  }

  if (awaiting_response_) {
    if (++response_waiting_counter_ > 5) {
      RCLCPP_ERROR(this->get_logger(), "Service change_to_autonomous timed out!");
      awaiting_response_ = false;
      awaiting_autonomous_ = false;
      pending_future_ =
        rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture();
      response_waiting_counter_ = 0;
    }
  }
}

std::optional<std::set<int64_t>> Level4ModeManager::parse_level4_tag(const std::string & str)
{
  std::set<int64_t> result;
  std::istringstream iss(str);
  std::string token;

  while (std::getline(iss, token, ',')) {
    if (token.empty()) {
      return std::nullopt;
    }

    for (char c : token) {
      if (!std::isdigit(c)) {
        return std::nullopt;
      }
    }

    try {
      int64_t value = std::stoll(token);
      result.insert(value);
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  return result;
}

bool Level4ModeManager::check_level4_availability(const Route & route)
{
  if (!is_initialized_) {
    RCLCPP_ERROR(this->get_logger(), "level4_mode_manager is not initialized.");
    return false;
  }

  const int64_t start_id = route.data.front().segments.front().preferred.id;
  if (!lanelet_map_ptr_->laneletLayer.exists(start_id)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot find the start lanelet in the map!!");
    return false;
  }

  lanelet::ConstLanelet start_lanelet = lanelet_map_ptr_->laneletLayer.get(start_id);
  if (!start_lanelet.hasAttribute(level_defining_tag)) {
    // Maybe it is a road_sholder, find from start pose
    const auto start_candidates = get_candidate_lanelets_from_pose(route.data.front().start);

    if (start_candidates.empty()) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "No Level 4 tag found in the starting lanelet. Level 4 is not available.");
      return false;
    }

    if (start_candidates.size() > 1) {
      RCLCPP_INFO(
        this->get_logger(),
        "Too many lanelets with a Level 4 tag detected. Unable to judge which to use. Level 4 is "
        "not available.");
      return false;
    }

    start_lanelet = start_candidates.front();
  }

  const auto level4_goal_candidates =
    parse_level4_tag(start_lanelet.attribute(level_defining_tag).value());
  if (!level4_goal_candidates) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid value in the Level 4 tag!!. Level 4 is not available.");
    return false;
  }

  bool goal_pose_found_on_indicated_lanelet = false;
  for (const int64_t id : *level4_goal_candidates) {
    if (!lanelet_map_ptr_->laneletLayer.exists(id)) {
      continue;
    }
    const lanelet::ConstLanelet lane = lanelet_map_ptr_->laneletLayer.get(id);

    if (autoware::experimental::lanelet2_utils::is_in_lanelet(route.data.front().goal, lane, 0.0)) {
      goal_pose_found_on_indicated_lanelet = true;
      break;
    }
  }

  if (!goal_pose_found_on_indicated_lanelet) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The goal pose is not on the lanelet specified by the Level 4 tag!! Level 4 is not "
      "available.");
    return false;
  }

  RCLCPP_DEBUG(this->get_logger(), "Level 4 driving is available for this route.");
  return true;
}

lanelet::ConstLanelets Level4ModeManager::get_candidate_lanelets_from_pose(
  const geometry_msgs::msg::Pose & pose)
{
  lanelet::ConstLanelets result;

  lanelet::BasicPoint2d point_2d(pose.position.x, pose.position.y);
  auto func = [&result, &point_2d](
                const lanelet::BoundingBox2d & box, const lanelet::ConstLanelet & lane) {
    if (
      boost::geometry::covered_by(point_2d, lane.polygon2d().basicPolygon()) &&
      lane.hasAttribute(level_defining_tag)) {
      result.push_back(lane);
    }
    if (result.size() > 1) {  // Having multiple candidates are wrong
      return true;
    }
    if (boost::geometry::distance(box, point_2d) > 5.0) {  // End of search (in meters)
      return true;
    }
    return false;
  };
  lanelet_map_ptr_->laneletLayer.nearestUntil(point_2d, func);
  return result;
}

}  // namespace autoware::level4_mode_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::level4_mode_manager::Level4ModeManager)
