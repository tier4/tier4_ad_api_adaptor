#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/msg/engage_status.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalEngagePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = proxy.create_service<tier4_external_api_msgs::srv::Engage>(
      "/api/external/set/engage",
      std::bind(&ExternalEngagePlugin::setEngage, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    cli_ = proxy.create_client<tier4_external_api_msgs::srv::Engage>(
      "/api/autoware/set/engage");
    cli_operator_ = proxy.create_client<tier4_external_api_msgs::srv::SetOperator>(
      "/api/autoware/set/operator");
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::EngageStatus>(
      "/api/external/get/engage", 1);
    node_ptr->declare_parameter("auto_operator_change", false);
    auto_operator_change_ = node_ptr->get_parameter("auto_operator_change").as_bool();
  }

  void on_timer() override
  {
    auto & data = *get_api_data();
    tier4_external_api_msgs::msg::EngageStatus status;
    if (data.engage_status) {
      status.engage = data.engage_status->engage;
    }
    if (data.autoware_state) {
      status.operation_mode = data.autoware_state->state;
    }
    pub_->publish(status);

    if (auto_operator_change_ && data.engage_status && data.external_operator) {
      // auto-operator logic: set operator when engage changes
    }
  }

  void setEngage(
    const tier4_external_api_msgs::srv::Engage::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::Engage::Response::SharedPtr response)
  {
    if (!cli_->service_is_ready()) {
      response->status = tier4_api_utils::response_error("service not ready");
      return;
    }
    auto result = cli_->async_send_request(
      std::make_shared<tier4_external_api_msgs::srv::Engage::Request>(*request)).get();
    response->status = result->status;
  }

private:
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::Engage>::SharedPtr srv_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr cli_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::SetOperator>::SharedPtr cli_operator_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::EngageStatus>::SharedPtr pub_;
  bool auto_operator_change_{false};
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalEngagePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
