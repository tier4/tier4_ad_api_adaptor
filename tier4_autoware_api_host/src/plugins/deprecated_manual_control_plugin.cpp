#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>

namespace tier4_autoware_api_host::plugin
{

class DeprecatedManualControlPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    node_ptr->declare_parameter("manual_control.mode", "local");
    auto mode = node_ptr->get_parameter("manual_control.mode").as_string();

    pub_heartbeat_ = node_ptr->create_publisher<autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat>(
      "/external/" + mode + "/heartbeat", 1);
    pub_pedals_ = node_ptr->create_publisher<autoware_adapi_v1_msgs::msg::PedalsCommand>(
      "/external/" + mode + "/pedals_cmd", 1);
    pub_steering_ = node_ptr->create_publisher<autoware_adapi_v1_msgs::msg::SteeringCommand>(
      "/external/" + mode + "/steering_cmd", 1);
    pub_gear_ = node_ptr->create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
      "/external/" + mode + "/gear_cmd", 1);
    pub_turn_indicators_ = node_ptr->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/external/" + mode + "/turn_indicators_cmd", 1);
    pub_hazard_lights_ = node_ptr->create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "/external/" + mode + "/hazard_lights_cmd", 1);
  }

private:
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::PedalsCommand>::SharedPtr pub_pedals_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::SteeringCommand>::SharedPtr pub_steering_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr pub_hazard_lights_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::DeprecatedManualControlPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
