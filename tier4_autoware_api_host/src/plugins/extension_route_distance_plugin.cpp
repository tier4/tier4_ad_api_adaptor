#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/route_distance.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExtensionRouteDistancePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::RouteDistance>(
      "/api/external/get/route_distance", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.route_distance) {
      return;
    }
    tier4_external_api_msgs::msg::RouteDistance msg;
    msg.stamp = data.route_distance->stamp;
    msg.remaining_distance = data.route_distance->data;
    pub_->publish(msg);
  }

private:
  rclcpp::Publisher<tier4_external_api_msgs::msg::RouteDistance>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExtensionRouteDistancePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
