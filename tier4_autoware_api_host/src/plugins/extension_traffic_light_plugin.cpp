#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/traffic_light_group.hpp>

namespace tier4_autoware_api_host::plugin
{

namespace
{
auto convert(const autoware_perception_msgs::msg::TrafficLightElement & internal)
{
  tier4_external_api_msgs::msg::TrafficLightElement external;
  external.color = internal.color;
  external.shape = internal.shape;
  external.status = internal.status;
  external.confidence = internal.confidence;
  return external;
}
}  // namespace

class ExtensionTrafficLightPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::TrafficLightGroup>(
      "/api/external/get/nearest_traffic_light_group", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.traffic_light_group) {
      return;
    }
    tier4_external_api_msgs::msg::TrafficLightGroup external;
    external.traffic_light_group_id = data.traffic_light_group->traffic_light_group_id;
    for (const auto & element : data.traffic_light_group->elements) {
      external.elements.push_back(convert(element));
    }
    pub_->publish(external);
  }

private:
  rclcpp::Publisher<tier4_external_api_msgs::msg::TrafficLightGroup>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExtensionTrafficLightPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
