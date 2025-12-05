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

#ifndef CREEP_GUIDANCE_CONTROLLER_HPP_
#define CREEP_GUIDANCE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <tier4_creep_guidance_msgs/msg/command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_status_array.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_command.hpp>
#include <tier4_creep_guidance_msgs/msg/creep_trigger_response.hpp>
#include <tier4_creep_guidance_msgs/msg/module.hpp>
#include <tier4_creep_guidance_msgs/msg/state.hpp>
#include <tier4_creep_guidance_msgs/srv/creep_trigger_command.hpp>

#include <memory>
#include <string>
#include <vector>

using CreepTriggerCommandSrv = tier4_creep_guidance_msgs::srv::CreepTriggerCommand;
using CreepStatusArray = tier4_creep_guidance_msgs::msg::CreepStatusArray;
using CreepStatus = tier4_creep_guidance_msgs::msg::CreepStatus;
using CreepTriggerCommand = tier4_creep_guidance_msgs::msg::CreepTriggerCommand;
using CreepTriggerResponse = tier4_creep_guidance_msgs::msg::CreepTriggerResponse;
using Module = tier4_creep_guidance_msgs::msg::Module;
using Command = tier4_creep_guidance_msgs::msg::Command;
using State = tier4_creep_guidance_msgs::msg::State;

/**
 * @brief Manages a single creep guidance module (crosswalk, intersection, or
 * intersection_occlusion)
 *
 * This class handles communication with a specific creep guidance planning module,
 * subscribing to its status and forwarding trigger commands to it.
 */
class CreepGuidanceModule
{
public:
  std::vector<CreepStatus> module_statuses_;
  rclcpp::Subscription<CreepStatusArray>::SharedPtr module_sub_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  tier4_api_utils::Client<CreepTriggerCommandSrv>::SharedPtr cli_set_module_;
  rclcpp::Logger logger_;

  /**
   * @brief Construct a new Creep Guidance Module object
   * @param node Pointer to the parent ROS 2 node
   * @param name Name of the module (e.g., "crosswalk", "intersection", "intersection_occlusion")
   */
  CreepGuidanceModule(rclcpp::Node * node, const std::string & name);

  /**
   * @brief Callback function to receive and store creep status messages from the planning module
   * @param message Received creep status array message
   */
  void module_callback(const CreepStatusArray::ConstSharedPtr message);

  /**
   * @brief Insert this module's statuses into the provided vector
   * @param creep_statuses Vector to insert statuses into
   */
  void insert_message(std::vector<CreepStatus> & creep_statuses);

  /**
   * @brief Call the creep trigger command service for this module
   * @param request Service request containing trigger commands
   * @param responses Service response to accumulate results
   */
  void call_service(
    CreepTriggerCommandSrv::Request::SharedPtr request,
    const CreepTriggerCommandSrv::Response::SharedPtr & responses);
};

namespace external_api
{
/**
 * @brief External API node for managing creep guidance across multiple modules
 *
 * This node acts as an external API interface for the creep guidance system.
 * It manages three creep guidance modules (crosswalk, intersection, and intersection_occlusion),
 * aggregates their statuses, and forwards trigger commands from external sources to the
 * appropriate planning modules.
 *
 * Published Topics:
 * - /api/external/get/creep_status (CreepStatusArray): Aggregated creep status from all modules
 *
 * Services:
 * - /api/external/set/creep_trigger_commands (CreepTriggerCommand): Accepts trigger commands
 *   and forwards them to the appropriate module
 */
class CreepGuidanceController : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Creep Guidance Controller object
   * @param options ROS 2 node options
   */
  explicit CreepGuidanceController(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<CreepGuidanceModule> crosswalk_;
  std::unique_ptr<CreepGuidanceModule> intersection_;
  std::unique_ptr<CreepGuidanceModule> intersection_occlusion_;

  /* publishers */
  rclcpp::Publisher<CreepStatusArray>::SharedPtr creep_status_pub_;

  /* service from external */
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<CreepTriggerCommandSrv>::SharedPtr srv_set_creep_trigger_;

  /* Timer */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Service callback to handle creep trigger commands from external API
   *
   * Routes each command to the appropriate module (crosswalk, intersection, or
   * intersection_occlusion) based on the module type specified in the request.
   *
   * @param requests Service request containing trigger commands
   * @param responses Service response containing results from each module
   */
  void set_creep_trigger(
    const CreepTriggerCommandSrv::Request::SharedPtr requests,
    const CreepTriggerCommandSrv::Response::SharedPtr responses);

  /**
   * @brief Timer callback to aggregate and publish creep status from all modules
   *
   * Called periodically (100ms) to collect status from all modules, sort them by
   * start_distance, and publish the aggregated status to the external API.
   */
  void on_timer();
};

}  // namespace external_api

#endif  // CREEP_GUIDANCE_CONTROLLER_HPP_
