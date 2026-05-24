#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"
#include "tier4_autoware_api_host/converter/routing.hpp"
#include "tier4_autoware_api_host/converter/response_status.hpp"

#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware/adapi_specs/routing.hpp>
#include <tier4_external_api_msgs/msg/route.hpp>
#include <tier4_external_api_msgs/srv/clear_route.hpp>
#include <tier4_external_api_msgs/srv/set_route.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalRoutePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_route_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::Route>(
      "/api/external/get/route", rclcpp::QoS(1).transient_local());
    srv_set_route_ = proxy.create_service<tier4_external_api_msgs::srv::SetRoute>(
      "/api/external/set/route",
      std::bind(&ExternalRoutePlugin::setRoute, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    srv_clear_route_ = proxy.create_service<tier4_external_api_msgs::srv::ClearRoute>(
      "/api/external/set/clear_route",
      std::bind(&ExternalRoutePlugin::clearRoute, this, _1, _2),
      rmw_qos_profile_services_default, group_);

    const autoware::component_interface_utils::NodeAdaptor adaptor(node_ptr);
    adaptor.init_cli(cli_set_route_);
    adaptor.init_cli(cli_clear_route_);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (data.adapi_route && !data.adapi_route->data.empty()) {
      pub_route_->publish(external_api::converter::convert(*data.adapi_route));
    }
  }

private:
  void setRoute(
    const tier4_external_api_msgs::srv::SetRoute::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetRoute::Response::SharedPtr response)
  {
    {
      const auto req = std::make_shared<tier4_external_api_msgs::srv::ClearRoute::Request>();
      const auto res = std::make_shared<tier4_external_api_msgs::srv::ClearRoute::Response>();
      clearRoute(req, res);
    }
    try {
      const auto req =
        std::make_shared<autoware::adapi_specs::routing::SetRoute::Service::Request>();
      *req = external_api::converter::convert(*request);
      const auto res = cli_set_route_->call(req);
      response->status = external_api::converter::convert(res->status);
    } catch (const autoware::component_interface_utils::ServiceException & error) {
      response->status = tier4_api_utils::response_error(error.what());
    }
  }

  void clearRoute(
    const tier4_external_api_msgs::srv::ClearRoute::Request::SharedPtr,
    const tier4_external_api_msgs::srv::ClearRoute::Response::SharedPtr response)
  {
    try {
      const auto req =
        std::make_shared<autoware::adapi_specs::routing::ClearRoute::Service::Request>();
      const auto res = cli_clear_route_->call(req);
      response->status = external_api::converter::convert(res->status);
    } catch (const autoware::component_interface_utils::ServiceException & error) {
      response->status = tier4_api_utils::response_error(error.what());
    }
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Route>::SharedPtr pub_route_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetRoute>::SharedPtr srv_set_route_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::ClearRoute>::SharedPtr srv_clear_route_;
  autoware::component_interface_utils::Client<autoware::adapi_specs::routing::SetRoute>::SharedPtr
    cli_set_route_;
  autoware::component_interface_utils::Client<autoware::adapi_specs::routing::ClearRoute>::SharedPtr
    cli_clear_route_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalRoutePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
