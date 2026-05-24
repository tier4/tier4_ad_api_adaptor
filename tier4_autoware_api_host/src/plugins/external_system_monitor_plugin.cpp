#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/system_monitor.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalSystemMonitorPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::SystemMonitor>(
      "/api/external/get/system_monitor", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (
      !data.cpu_temperature || !data.memory_status || !data.gpu_status || !data.network_status ||
      !data.hdd_status) {
      return;
    }
    const std::string hostname = data.cpu_temperature->hostname;
    tier4_external_api_msgs::msg::SystemMonitor msg;
    msg.hostname = hostname;
    msg.cpu_temperature = *data.cpu_temperature;
    msg.memory_status = *data.memory_status;
    msg.gpu_status = *data.gpu_status;
    msg.network_status = *data.network_status;
    msg.hdd_status = *data.hdd_status;
    pub_->publish(msg);
  }

private:
  rclcpp::Publisher<tier4_external_api_msgs::msg::SystemMonitor>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalSystemMonitorPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
