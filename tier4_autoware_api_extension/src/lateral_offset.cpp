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

#include "lateral_offset.hpp"

#include <tier4_external_api_msgs/msg/response_status.hpp>

#include <memory>

namespace tier4_autoware_api_extension
{

LateralOffset::LateralOffset(const rclcpp::NodeOptions & options) : Node("lateral_offset", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_ = create_service<ExternalService>(
    "/api/external/set/lateral_offset", std::bind(&LateralOffset::on_service, this, _1, _2));
  cli_ = create_client<PlanningService>(
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/"
    "set_lateral_offset");
}

void LateralOffset::on_service(
  const std::shared_ptr<rmw_request_id_t> header, const ExternalService::Request::SharedPtr request)
{
  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;

  if (!cli_->service_is_ready()) {
    ExternalService::Response response;
    response.status.code = ResponseStatus::ERROR;
    response.status.message = "Internal service is not available.";
    srv_->send_response(*header, response);
    return;
  }

  // Direct copy of the request fields to the planning service request.
  const auto planning_request = std::make_shared<PlanningService::Request>();
  planning_request->shift_mode = request->shift_mode;
  planning_request->shift_value = request->shift_value;
  planning_request->shift_direction_value = request->shift_direction_value;

  // Forward the downstream response to the external caller via the client
  // callback.
  cli_->async_send_request(
    planning_request, [this, header](rclcpp::Client<PlanningService>::SharedFuture future) {
      const auto & planning_response = future.get();

      // Direct copy of the planning service response to the external response.
      ExternalService::Response response;
      response.status.code = planning_response->status.code;
      response.status.message = planning_response->status.message;
      srv_->send_response(*header, response);
    });
}

}  // namespace tier4_autoware_api_extension

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_autoware_api_extension::LateralOffset)
