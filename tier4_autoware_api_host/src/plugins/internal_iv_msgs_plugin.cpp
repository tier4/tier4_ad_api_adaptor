#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <tier4_auto_msgs_converter/tier4_auto_msgs_converter.hpp>
#include <tier4_system_msgs/msg/autoware_state.hpp>

namespace tier4_autoware_api_host::plugin
{

class InternalIVMsgsPlugin : public Tier4ApiAdaptorPluginBase
{
  using AutowareStateInput = autoware_system_msgs::msg::AutowareState;
  using AutowareStateOutput = tier4_system_msgs::msg::AutowareState;
  using EmergencyStateInput = autoware_adapi_v1_msgs::msg::MrmState;
  using TrajectoryInput = autoware_planning_msgs::msg::Trajectory;
  using TrajectoryOutput = tier4_planning_msgs::msg::Trajectory;
  using TrackedObjectsInput = autoware_perception_msgs::msg::TrackedObjects;
  using DynamicObjectsOutput = tier4_perception_msgs::msg::DynamicObjectArray;

public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_state_ = node_ptr->create_publisher<AutowareStateOutput>(
      "/api/iv_msgs/autoware/state", 1);
    pub_trajectory_ = node_ptr->create_publisher<TrajectoryOutput>(
      "/api/iv_msgs/planning/scenario_planning/trajectory", 1);
    pub_dynamic_objects_ = node_ptr->create_publisher<DynamicObjectsOutput>(
      "/api/iv_msgs/perception/object_recognition/tracking/objects", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (data.autoware_state) {
      auto state = tier4_auto_msgs_converter::convert(*data.autoware_state);
      if (data.mrm_state && data.mrm_state->state != EmergencyStateInput::NORMAL) {
        state.state = AutowareStateOutput::EMERGENCY;
      }
      pub_state_->publish(state);
    }
    if (data.autoware_trajectory) {
      pub_trajectory_->publish(tier4_auto_msgs_converter::convert(*data.autoware_trajectory));
    }
    if (data.tracked_objects) {
      pub_dynamic_objects_->publish(tier4_auto_msgs_converter::convert(*data.tracked_objects));
    }
  }

private:
  rclcpp::Publisher<AutowareStateOutput>::SharedPtr pub_state_;
  rclcpp::Publisher<TrajectoryOutput>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<DynamicObjectsOutput>::SharedPtr pub_dynamic_objects_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::InternalIVMsgsPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
