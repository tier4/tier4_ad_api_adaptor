#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/classified_diagnostics.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalDiagnosticsPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::ClassifiedDiagnostics>(
      "/api/external/get/diagnostics", 1);
  }

  void on_timer() override
  {
    tier4_external_api_msgs::msg::ClassifiedDiagnostics msg;
    msg.stamp = node_ptr_->now();
    pub_->publish(msg);
  }

private:
  rclcpp::Publisher<tier4_external_api_msgs::msg::ClassifiedDiagnostics>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalDiagnosticsPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
