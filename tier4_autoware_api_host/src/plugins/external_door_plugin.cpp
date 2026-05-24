#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/door_status.hpp>
#include <tier4_external_api_msgs/srv/set_door.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalDoorPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = proxy.create_service<tier4_external_api_msgs::srv::SetDoor>(
      "/api/external/set/door",
      std::bind(&ExternalDoorPlugin::setDoor, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    cli_ = proxy.create_client<tier4_external_api_msgs::srv::SetDoor>(
      "/api/vehicle/set/door");
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::DoorStatus>(
      "/api/external/get/door", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.door_status) {
      return;
    }
    tier4_external_api_msgs::msg::DoorStatus msg;
    msg.stamp = node_ptr_->now();
    msg.status = data.door_status->status;
    pub_->publish(msg);
  }

private:
  void setDoor(
    const tier4_external_api_msgs::srv::SetDoor::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetDoor::Response::SharedPtr response)
  {
    const auto [status, resp] = cli_->call(request);
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetDoor>::SharedPtr srv_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::SetDoor>::SharedPtr cli_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::DoorStatus>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalDoorPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
