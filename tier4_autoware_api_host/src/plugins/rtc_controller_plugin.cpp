#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "adaptors/rtc_controller_host.hpp"

#include <memory>

namespace tier4_autoware_api_host::plugin
{

class RTCControllerPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    host_ = std::make_unique<adaptors::RtcControllerHost>(node_ptr);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);
    srv_set_rtc_ = proxy.create_service<tier4_rtc_msgs::srv::CooperateCommands>(
      "/api/external/set/rtc_commands",
      [this](auto req, auto res) { host_->set_rtc(req, res); },
      rmw_qos_profile_services_default, host_->callback_group());
    srv_set_auto_mode_ = proxy.create_service<tier4_rtc_msgs::srv::AutoModeWithModule>(
      "/api/external/set/rtc_auto_mode",
      [this](auto req, auto res) { host_->set_rtc_auto_mode(req, res); },
      rmw_qos_profile_services_default, host_->callback_group());
  }

  void on_timer() override
  {
    host_->on_timer();
    host_->on_auto_mode_timer();
  }

private:
  std::unique_ptr<adaptors::RtcControllerHost> host_;
  tier4_api_utils::Service<tier4_rtc_msgs::srv::CooperateCommands>::SharedPtr srv_set_rtc_;
  tier4_api_utils::Service<tier4_rtc_msgs::srv::AutoModeWithModule>::SharedPtr srv_set_auto_mode_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::RTCControllerPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
