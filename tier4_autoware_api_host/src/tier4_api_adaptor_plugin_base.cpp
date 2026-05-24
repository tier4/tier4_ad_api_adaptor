#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"

#include "tier4_autoware_api_host/api_data.hpp"

namespace tier4_autoware_api_host::plugin
{

void Tier4ApiAdaptorPluginBase::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<ApiData> & api_data)
{
  name_ = name;
  node_ptr_ = node_ptr;
  api_data_ = api_data;
}

rcl_interfaces::msg::SetParametersResult Tier4ApiAdaptorPluginBase::on_parameter(
  const std::vector<rclcpp::Parameter> & /*params*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace tier4_autoware_api_host::plugin
