#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/hazard_status_stamped.hpp>

namespace tier4_autoware_api_host::plugin
{

class DeprecatedHazardStatusPlugin : public Tier4ApiAdaptorPluginBase
{
  using InternalMessage = autoware_system_msgs::msg::HazardStatusStamped;
  using ExternalMessage = tier4_external_api_msgs::msg::HazardStatusStamped;

public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<ExternalMessage>("/api/external/get/hazard_status", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.hazard_status) {
      return;
    }
    const auto & internal = *data.hazard_status;
    ExternalMessage external;
    external.stamp = internal.stamp;
    external.status.level = internal.status.level;
    external.status.emergency = internal.status.emergency;
    external.status.emergency_holding = internal.status.emergency_holding;
    external.status.diag_no_fault = internal.status.diag_no_fault;
    external.status.diag_safe_fault = internal.status.diag_safe_fault;
    external.status.diag_latent_fault = internal.status.diag_latent_fault;
    external.status.diag_single_point_fault = internal.status.diag_single_point_fault;
    pub_->publish(external);
  }

private:
  rclcpp::Publisher<ExternalMessage>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::DeprecatedHazardStatusPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
