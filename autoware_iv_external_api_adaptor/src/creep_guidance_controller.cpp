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

#include "creep_guidance_controller.hpp"

#include <memory>

namespace
{
// Temporary fix for ROS2 humble
void check_inf_distance(CreepStatus & status)
{
  if (!std::isfinite(status.start_distance)) {
    status.start_distance = -100000.0;
  }
  if (!std::isfinite(status.finish_distance)) {
    status.finish_distance = -100000.0;
  }
}

void align_creep_status_array(std::vector<CreepStatus> & statuses_vector)
{
  if (statuses_vector.empty()) {
    return;
  }
  tier4_creep_guidance_msgs::msg::CreepStatus current_status;
  check_inf_distance(statuses_vector[0]);
  for (size_t i = 1; i < statuses_vector.size(); i++) {
    check_inf_distance(statuses_vector[i]);
    current_status = statuses_vector[i];
    int j = i - 1;

    while (j >= 0 && current_status.start_distance < statuses_vector[j].start_distance) {
      statuses_vector[j + 1] = statuses_vector[j];
      j = j - 1;
    }
    statuses_vector[j + 1] = current_status;
  }
}
}  // namespace

CreepGuidanceModule::CreepGuidanceModule(rclcpp::Node * node, const std::string & name)
: logger_(node->get_logger())
{
  using namespace std::literals::chrono_literals;
  using std::placeholders::_1;
  tier4_api_utils::ServiceProxyNodeInterface proxy(node);

  module_sub_ = node->create_subscription<CreepStatusArray>(
    "/planning/creep_guidance_status/" + name, rclcpp::QoS(1),
    std::bind(&CreepGuidanceModule::module_callback, this, _1));

  // // Create a reentrant callback group for the service client to avoid deadlock
  // client_callback_group_ =
  //   node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  cli_set_module_ = proxy.create_client<CreepTriggerCommandSrv>(
    "/planning/creep_guidance_commands/" + name, rmw_qos_profile_services_default);
}

void CreepGuidanceModule::module_callback(const CreepStatusArray::ConstSharedPtr message)
{
  module_statuses_ = message->statuses;
}

void CreepGuidanceModule::insert_message(std::vector<CreepStatus> & creep_statuses)
{
  creep_statuses.insert(creep_statuses.end(), module_statuses_.begin(), module_statuses_.end());
}

void CreepGuidanceModule::call_service(
  CreepTriggerCommandSrv::Request::SharedPtr request,
  const CreepTriggerCommandSrv::Response::SharedPtr & responses)
{
  using namespace std::literals::chrono_literals;
  // Increase timeout to 10 seconds to wait for server response
  const auto [status, resp] = cli_set_module_->call(request);
  if (!tier4_api_utils::is_success(status)) {
    RCLCPP_ERROR(
      logger_, "Failed to call service: %s (code: %d)", status.message.c_str(), status.code);
    return;
  }
  responses->responses.insert(
    responses->responses.end(), resp->responses.begin(), resp->responses.end());
}

namespace external_api
{
CreepGuidanceController::CreepGuidanceController(const rclcpp::NodeOptions & options)
: Node("external_api_creep_guidance_controller", options)
{
  using namespace std::literals::chrono_literals;
  using std::placeholders::_1;
  using std::placeholders::_2;
  tier4_api_utils::ServiceProxyNodeInterface proxy(this);

  crosswalk_ = std::make_unique<CreepGuidanceModule>(this, "crosswalk");
  intersection_ = std::make_unique<CreepGuidanceModule>(this, "intersection");
  intersection_occlusion_ = std::make_unique<CreepGuidanceModule>(this, "intersection_occlusion");

  creep_status_pub_ =
    create_publisher<CreepStatusArray>("/api/external/get/creep_status", rclcpp::QoS(1));

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_creep_trigger_ = proxy.create_service<CreepTriggerCommandSrv>(
    "/api/external/set/creep_trigger_commands",
    std::bind(&CreepGuidanceController::set_creep_trigger, this, _1, _2),
    rmw_qos_profile_services_default, group_);

  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&CreepGuidanceController::on_timer, this));
}

void CreepGuidanceController::on_timer()
{
  std::vector<CreepStatus> creep_statuses;
  crosswalk_->insert_message(creep_statuses);
  intersection_->insert_message(creep_statuses);
  intersection_occlusion_->insert_message(creep_statuses);

  align_creep_status_array(creep_statuses);

  CreepStatusArray msg;
  msg.stamp = now();
  msg.statuses = creep_statuses;
  creep_status_pub_->publish(msg);
}

void CreepGuidanceController::set_creep_trigger(
  const CreepTriggerCommandSrv::Request::SharedPtr requests,
  const CreepTriggerCommandSrv::Response::SharedPtr responses)
{
  for (tier4_creep_guidance_msgs::msg::CreepTriggerCommand & command : requests->commands) {
    auto request = std::make_shared<CreepTriggerCommandSrv::Request>();
    request->stamp = requests->stamp;
    request->commands = {command};
    switch (command.module.type) {
      case Module::CROSSWALK: {
        crosswalk_->call_service(request, responses);
        break;
      }
      case Module::INTERSECTION: {
        intersection_->call_service(request, responses);
        break;
      }
      case Module::INTERSECTION_OCCLUSION: {
        intersection_occlusion_->call_service(request, responses);
        break;
      }
      case Module::NONE:
      default: {
        // Unknown module type, add a failure response
        CreepTriggerResponse failure;
        failure.id = command.id;
        failure.module = command.module;
        failure.success = false;
        responses->responses.push_back(failure);
        break;
      }
    }
  }
}

}  // namespace external_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::CreepGuidanceController)
