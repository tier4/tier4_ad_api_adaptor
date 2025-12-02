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

#ifndef INITIAL_POSE_HPP_
#define INITIAL_POSE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <tier4_external_api_msgs/srv/initialize_pose.hpp>

#include <memory>

namespace tier4_deprecated_api_adapter
{

class InitialPose : public rclcpp::Node
{
public:
  explicit InitialPose(const rclcpp::NodeOptions & options);

private:
  using NewService = autoware_adapi_v1_msgs::srv::InitializeLocalization;
  using OldService = tier4_external_api_msgs::srv::InitializePose;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<OldService>::SharedPtr srv_;
  rclcpp::Client<NewService>::SharedPtr cli_;

  void on_service(
    const std::shared_ptr<rmw_request_id_t> header, const OldService::Request::SharedPtr request);
};

}  // namespace tier4_deprecated_api_adapter

#endif  // INITIAL_POSE_HPP_
