#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/awapi_info_builder.hpp"

#include <awapi_awiv_adapter/awapi_vehicle_state_publisher.hpp>
#include <memory>

namespace tier4_autoware_api_host::plugin
{

class AwapiVehicleStatusPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    publisher_ = std::make_unique<autoware_api::AutowareIvVehicleStatePublisher>(*node_ptr);
  }

  void on_timer() override
  {
    publisher_->statePublisher(build_awapi_info(*get_api_data()));
  }

private:
  std::unique_ptr<autoware_api::AutowareIvVehicleStatePublisher> publisher_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::AwapiVehicleStatusPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
