#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/srv/set_service.hpp>
#include <tier4_external_api_msgs/msg/service.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalServicePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = proxy.create_service<tier4_external_api_msgs::srv::SetService>(
      "/api/external/set/service",
      std::bind(&ExternalServicePlugin::setService, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::Service>(
      "/api/external/get/service", rclcpp::QoS{1}.transient_local());
  }

private:
  void setService(
    const tier4_external_api_msgs::srv::SetService::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetService::Response::SharedPtr response)
  {
    response->status = tier4_api_utils::response_success();
    pub_->publish(request->mode);
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetService>::SharedPtr srv_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Service>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalServicePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
