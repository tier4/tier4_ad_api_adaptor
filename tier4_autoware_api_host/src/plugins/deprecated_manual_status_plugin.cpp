#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"

#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/vehicle_command_stamped.hpp>

namespace tier4_autoware_api_host::plugin
{

class DeprecatedManualStatusPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    const auto bind = [this](auto && func) {
      return std::bind(func, this, std::placeholders::_1);
    };

    sub_control_command_ = node_ptr->create_subscription<autoware_control_msgs::msg::Control>(
      "/external/selected/control_cmd", 1, bind(&DeprecatedManualStatusPlugin::relay_control_command));
    sub_selected_pedals_ = node_ptr->create_subscription<autoware_adapi_v1_msgs::msg::PedalsCommand>(
      "/external/selected/pedals_cmd", 1, bind(&DeprecatedManualStatusPlugin::relay_pedals));
    sub_selected_steering_ = node_ptr->create_subscription<autoware_adapi_v1_msgs::msg::SteeringCommand>(
      "/external/selected/steering_cmd", 1, bind(&DeprecatedManualStatusPlugin::relay_steering));

    pub_vehicle_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::VehicleCommandStamped>(
      "/api/external/get/command/selected/vehicle", 1);
    pub_control_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::ControlCommandStamped>(
      "/api/external/get/command/selected/control", 1);
  }

private:
  void relay_control_command(const autoware_control_msgs::msg::Control & msg)
  {
    tier4_external_api_msgs::msg::VehicleCommandStamped command;
    command.stamp = msg.stamp;
    command.command.velocity = msg.longitudinal.velocity;
    command.command.acceleration = msg.longitudinal.acceleration;
    pub_vehicle_->publish(command);
  }

  void relay_pedals(const autoware_adapi_v1_msgs::msg::PedalsCommand & msg)
  {
    tier4_external_api_msgs::msg::ControlCommandStamped command;
    command.stamp = msg.stamp;
    command.control.throttle = msg.throttle;
    command.control.brake = msg.brake;
    command.control.steering_angle = steering_tire_angle_;
    command.control.steering_angle_velocity = steering_tire_velocity_;
    pub_control_->publish(command);
  }

  void relay_steering(const autoware_adapi_v1_msgs::msg::SteeringCommand & msg)
  {
    steering_tire_angle_ = msg.steering_tire_angle;
    steering_tire_velocity_ = msg.steering_tire_velocity;
  }

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_command_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::PedalsCommand>::SharedPtr sub_selected_pedals_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::SteeringCommand>::SharedPtr sub_selected_steering_;
  float steering_tire_angle_{0.0f};
  float steering_tire_velocity_{0.0f};
  rclcpp::Publisher<tier4_external_api_msgs::msg::VehicleCommandStamped>::SharedPtr pub_vehicle_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr pub_control_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::DeprecatedManualStatusPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
