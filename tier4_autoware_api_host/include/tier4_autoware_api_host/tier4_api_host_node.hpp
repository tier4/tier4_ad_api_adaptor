#ifndef TIER4_AUTOWARE_API_HOST__TIER4_API_HOST_NODE_HPP_
#define TIER4_AUTOWARE_API_HOST__TIER4_API_HOST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>

#include "api_data.hpp"
#include "tier4_api_adaptor_plugin_base.hpp"

#include <memory>
#include <string>
#include <vector>

namespace tier4_autoware_api_host
{

class Tier4ApiHostNode : public rclcpp::Node
{
public:
  explicit Tier4ApiHostNode(const rclcpp::NodeOptions & options);

private:
  void set_up_params();
  void initialize_plugins();
  void load_plugin(const std::string & plugin_name);
  bool is_plugin_enabled_for_api_mode(const std::string & plugin_name) const;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & params);

  void create_subscriptions();
  void create_services();
  void create_relays();
  void on_timer();

  int api_mode_{1};

  std::unique_ptr<pluginlib::ClassLoader<plugin::Tier4ApiAdaptorPluginBase>> plugin_loader_;
  std::vector<std::shared_ptr<plugin::Tier4ApiAdaptorPluginBase>> plugins_;
  std::shared_ptr<plugin::ApiData> api_data_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  bool initialized_plugins_{false};
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
};

}  // namespace tier4_autoware_api_host

#endif  // TIER4_AUTOWARE_API_HOST__TIER4_API_HOST_NODE_HPP_
