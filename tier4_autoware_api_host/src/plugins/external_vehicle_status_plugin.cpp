#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_auto_msgs_converter/tier4_auto_msgs_converter.hpp>
#include <tier4_external_api_msgs/msg/gear_shift.hpp>
#include <tier4_external_api_msgs/msg/turn_signal.hpp>
#include <tier4_external_api_msgs/msg/vehicle_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/vehicle_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/shift.hpp>
#include <tier4_vehicle_msgs/msg/turn_signal.hpp>

namespace
{

tier4_external_api_msgs::msg::TurnSignal convert(const tier4_vehicle_msgs::msg::TurnSignal & msg)
{
  using External = tier4_external_api_msgs::msg::TurnSignal;
  using Internal = tier4_vehicle_msgs::msg::TurnSignal;

  switch (msg.data) {
    case Internal::NONE:
      return tier4_external_api_msgs::build<External>().data(External::NONE);
    case Internal::LEFT:
      return tier4_external_api_msgs::build<External>().data(External::LEFT);
    case Internal::RIGHT:
      return tier4_external_api_msgs::build<External>().data(External::RIGHT);
    case Internal::HAZARD:
      return tier4_external_api_msgs::build<External>().data(External::HAZARD);
  }
  throw std::out_of_range("turn_signal=" + std::to_string(msg.data));
}

tier4_external_api_msgs::msg::GearShift convert(const tier4_vehicle_msgs::msg::Shift & msg)
{
  using External = tier4_external_api_msgs::msg::GearShift;
  using Internal = tier4_vehicle_msgs::msg::Shift;

  switch (msg.data) {
    case Internal::NONE:
      return tier4_external_api_msgs::build<External>().data(External::NONE);
    case Internal::PARKING:
      return tier4_external_api_msgs::build<External>().data(External::PARKING);
    case Internal::REVERSE:
      return tier4_external_api_msgs::build<External>().data(External::REVERSE);
    case Internal::NEUTRAL:
      return tier4_external_api_msgs::build<External>().data(External::NEUTRAL);
    case Internal::DRIVE:
      return tier4_external_api_msgs::build<External>().data(External::DRIVE);
    case Internal::LOW:
      return tier4_external_api_msgs::build<External>().data(External::LOW);
  }
  throw std::out_of_range("gear_shift=" + std::to_string(msg.data));
}

}  // namespace

namespace tier4_autoware_api_host::plugin
{

class ExternalVehicleStatusPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_status_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::VehicleStatusStamped>(
      "/api/external/get/vehicle/status", 1);
    pub_cmd_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::VehicleCommandStamped>(
      "/api/external/get/command/selected/vehicle", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    const auto subscriptions = {
      std::make_pair(std::static_pointer_cast<const void>(data.velocity), "velocity"),
      std::make_pair(std::static_pointer_cast<const void>(data.steer), "steering"),
      std::make_pair(std::static_pointer_cast<const void>(data.turn_indicators), "turn_indicators"),
      std::make_pair(std::static_pointer_cast<const void>(data.hazard_lights), "hazard_lights"),
      std::make_pair(std::static_pointer_cast<const void>(data.gear), "gear_shift")};

    for (const auto & [pointer, topic] : subscriptions) {
      if (!pointer) {
        RCLCPP_WARN_THROTTLE(
          node_ptr_->get_logger(), *node_ptr_->get_clock(), 5000,
          "The %s topic is not subscribed", topic);
        return;
      }
    }

    using namespace tier4_auto_msgs_converter;  // NOLINT
    try {
      tier4_external_api_msgs::msg::VehicleStatusStamped msg;
      msg.stamp = node_ptr_->now();
      msg.status.twist.linear.x = data.velocity->longitudinal_velocity;
      msg.status.twist.linear.y = data.velocity->lateral_velocity;
      msg.status.twist.angular.z = data.velocity->heading_rate;
      msg.status.steering.data = convert(*data.steer).data;
      msg.status.turn_signal = convert(convert(*data.turn_indicators, *data.hazard_lights));
      msg.status.gear_shift = convert(convert(*data.gear).shift);
      pub_status_->publish(msg);
    } catch (const std::out_of_range & exception) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "%s", exception.what());
    }

    if (data.vehicle_cmd) {
      tier4_external_api_msgs::msg::VehicleCommandStamped cmd;
      cmd.stamp = data.vehicle_cmd->stamp;
      cmd.command.velocity = data.vehicle_cmd->longitudinal.velocity;
      cmd.command.acceleration = data.vehicle_cmd->longitudinal.acceleration;
      pub_cmd_->publish(cmd);
    }
  }

private:
  rclcpp::Publisher<tier4_external_api_msgs::msg::VehicleStatusStamped>::SharedPtr pub_status_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::VehicleCommandStamped>::SharedPtr pub_cmd_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalVehicleStatusPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
