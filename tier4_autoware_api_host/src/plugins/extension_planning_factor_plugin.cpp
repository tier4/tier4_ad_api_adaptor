#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"
#include "tier4_autoware_api_host/adaptors/planning_factor_convert.hpp"

#include <tier4_external_api_msgs/msg/planning_factor_array.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExtensionPlanningFactorPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::PlanningFactorArray>(
      "/api/external/get/planning_factors", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.planning_factors) {
      return;
    }
    tier4_external_api_msgs::msg::PlanningFactorArray external;
    external.stamp = node_ptr_->now();
    for (const auto & factor : data.planning_factors->factors) {
      external.factors.push_back(
        adaptors::convert(data.planning_factors->header, factor));
    }
    if (!external.factors.empty()) {
      pub_->publish(external);
    }
  }

private:
  rclcpp::Publisher<tier4_external_api_msgs::msg::PlanningFactorArray>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExtensionPlanningFactorPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
