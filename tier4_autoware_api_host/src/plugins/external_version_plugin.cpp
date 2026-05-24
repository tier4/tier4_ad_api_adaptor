#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <autoware_external_api_msgs/srv/get_version.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalVersionPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = proxy.create_service<autoware_external_api_msgs::srv::GetVersion>(
      "/api/external/get/version",
      std::bind(&ExternalVersionPlugin::getVersion, this, _1, _2),
      rmw_qos_profile_services_default, group_);
  }

private:
  void getVersion(
    const autoware_external_api_msgs::srv::GetVersion::Request::SharedPtr,
    const autoware_external_api_msgs::srv::GetVersion::Response::SharedPtr response)
  {
    response->version = "0.4.2";
    response->status = tier4_api_utils::response_success();
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<autoware_external_api_msgs::srv::GetVersion>::SharedPtr srv_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalVersionPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
