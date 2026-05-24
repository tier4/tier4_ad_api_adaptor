#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/rosbag_logging_mode.hpp>
#include <tier4_external_api_msgs/srv/set_rosbag_logging_mode.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalRosbagLoggingModePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = proxy.create_service<tier4_external_api_msgs::srv::SetRosbagLoggingMode>(
      "/api/external/set/rosbag_logging_mode",
      std::bind(&ExternalRosbagLoggingModePlugin::setMode, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    cli_ = proxy.create_client<tier4_external_api_msgs::srv::SetRosbagLoggingMode>(
      "/api/autoware/set/rosbag_logging_mode");
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::RosbagLoggingMode>(
      "/api/external/get/rosbag_logging_mode", 1);
  }

  void on_timer() override
  {
    if (get_api_data()->rosbag_logging_mode) {
      pub_->publish(*get_api_data()->rosbag_logging_mode);
    }
  }

private:
  void setMode(
    const tier4_external_api_msgs::srv::SetRosbagLoggingMode::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetRosbagLoggingMode::Response::SharedPtr response)
  {
    const auto [status, resp] =
      cli_->call(request, std::chrono::seconds(190));
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetRosbagLoggingMode>::SharedPtr srv_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::SetRosbagLoggingMode>::SharedPtr cli_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::RosbagLoggingMode>::SharedPtr pub_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalRosbagLoggingModePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
