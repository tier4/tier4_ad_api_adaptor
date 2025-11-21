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

#include "initial_pose.hpp"

#include "utils/response.hpp"

#include <array>
#include <memory>
#include <utility>

namespace tier4_deprecated_api_adapter
{

constexpr std::chrono::seconds initial_pose_timeout = std::chrono::seconds(300);

// clang-format off
const std::array<double, 36> particle_covariance =
{
  1.00, 0.00, 0.00, 0.00, 0.00,  0.00,
  0.00, 1.00, 0.00, 0.00, 0.00,  0.00,
  0.00, 0.00, 0.01, 0.00, 0.00,  0.00,
  0.00, 0.00, 0.00, 0.01, 0.00,  0.00,
  0.00, 0.00, 0.00, 0.00, 0.01,  0.00,
  0.00, 0.00, 0.00, 0.00, 0.00, 10.00,
};
// clang-format on

InitialPose::InitialPose(const rclcpp::NodeOptions & options)
: Node("external_api_initial_pose", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_ = create_service<OldService>(
    "/api/external/set/initialize_pose", std::bind(&InitialPose::on_service, this, _1, _2));
  cli_ = create_client<NewService>("/api/localization/initialize");
}

void InitialPose::on_service(
  const std::shared_ptr<rmw_request_id_t> header, const OldService::Request::SharedPtr request)
{
  const auto on_response = [this, header](rclcpp::Client<NewService>::SharedFuture future) {
    if (!timer_) return;  // The timer is reset when it is timed out.
    timer_->cancel();
    timer_.reset();
    const auto res = future.get();
    const auto function = res->status.success ? utils::response_success : utils::response_error;
    OldService::Response response;
    response.status = function(res->status.message);
    return srv_->send_response(*header, response);
  };

  const auto on_timeout = [this, header]() {
    timer_->cancel();
    timer_.reset();
    OldService::Response response;
    response.status = utils::response_error("Internal service has timed out.");
    return srv_->send_response(*header, response);
  };

  if (!cli_->service_is_ready()) {
    OldService::Response response;
    response.status = utils::response_error("Internal service is not available.");
    return srv_->send_response(*header, response);
  }

  const auto req = std::make_shared<NewService::Request>();
  req->pose.push_back(request->pose);
  req->pose.back().pose.covariance = particle_covariance;
  cli_->async_send_request(req, on_response);
  timer_ = rclcpp::create_timer(this, get_clock(), initial_pose_timeout, std::move(on_timeout));
}

}  // namespace tier4_deprecated_api_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_deprecated_api_adapter::InitialPose)
