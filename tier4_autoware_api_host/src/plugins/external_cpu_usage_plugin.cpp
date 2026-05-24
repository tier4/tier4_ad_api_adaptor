#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

namespace tier4_autoware_api_host::plugin
{

class ExternalCpuUsagePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::CpuUsage>(
      "/api/external/get/cpu_usage", 1);
  }

  void on_timer() override
  {
    auto & data = *get_api_data();
    if (data.cpu_usage) {
      pub_->publish(*data.cpu_usage);
    }
  }

private:
  rclcpp::Publisher<tier4_external_api_msgs::msg::CpuUsage>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalCpuUsagePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
