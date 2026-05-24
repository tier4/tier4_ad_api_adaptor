#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"

#include <tier4_external_api_msgs/srv/pause_driving.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalVelocityPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_pause_ = proxy.create_service<tier4_external_api_msgs::srv::PauseDriving>(
      "/api/external/set/pause_driving",
      std::bind(&ExternalVelocityPlugin::setPauseDriving, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    cli_pause_ = proxy.create_client<tier4_external_api_msgs::srv::PauseDriving>(
      "/api/autoware/set/pause_driving");
    srv_velocity_ = proxy.create_service<tier4_external_api_msgs::srv::SetVelocityLimit>(
      "/api/external/set/velocity_limit",
      std::bind(&ExternalVelocityPlugin::setVelocityLimit, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    cli_velocity_ = proxy.create_client<tier4_external_api_msgs::srv::SetVelocityLimit>(
      "/api/autoware/set/velocity_limit");
  }

private:
  void setPauseDriving(
    const tier4_external_api_msgs::srv::PauseDriving::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::PauseDriving::Response::SharedPtr response)
  {
    const auto [status, resp] = cli_pause_->call(request);
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  }

  void setVelocityLimit(
    const tier4_external_api_msgs::srv::SetVelocityLimit::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetVelocityLimit::Response::SharedPtr response)
  {
    const auto [status, resp] = cli_velocity_->call(request);
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::PauseDriving>::SharedPtr srv_pause_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::PauseDriving>::SharedPtr cli_pause_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetVelocityLimit>::SharedPtr srv_velocity_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::SetVelocityLimit>::SharedPtr cli_velocity_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalVelocityPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
