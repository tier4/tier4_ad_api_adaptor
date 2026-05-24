#ifndef TIER4_AUTOWARE_API_HOST__TIER4_API_ADAPTOR_PLUGIN_BASE_HPP_
#define TIER4_AUTOWARE_API_HOST__TIER4_API_ADAPTOR_PLUGIN_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace tier4_autoware_api_host::plugin
{

struct ApiData;

class Tier4ApiAdaptorPluginBase
{
public:
  virtual ~Tier4ApiAdaptorPluginBase() = default;

  virtual void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<ApiData> & api_data);

  virtual void on_timer() {}
  virtual rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & params);

  std::string get_name() const { return name_; }

protected:
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }
  std::shared_ptr<ApiData> get_api_data() const { return api_data_; }

  std::string name_{"unnamed_plugin"};
  rclcpp::Node * node_ptr_{nullptr};
  std::shared_ptr<ApiData> api_data_{nullptr};
};

}  // namespace tier4_autoware_api_host::plugin

#endif  // TIER4_AUTOWARE_API_HOST__TIER4_API_ADAPTOR_PLUGIN_BASE_HPP_
