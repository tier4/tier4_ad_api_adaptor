#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>
#include <tier4_external_api_msgs/msg/operator.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalStartPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = proxy.create_service<std_srvs::srv::Trigger>(
      "/api/autoware/set/start_request",
      std::bind(&ExternalStartPlugin::setRequestStart, this, _1, _2),
      rmw_qos_profile_services_default, group_);

    sub_operator_ = node_ptr->create_subscription<tier4_external_api_msgs::msg::Operator>(
      "/api/external/get/operator", rclcpp::QoS(1),
      std::bind(&ExternalStartPlugin::onOperator, this, _1));
  }

private:
  void setRequestStart(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    using Operator = tier4_external_api_msgs::msg::Operator;

    if (operator_ && operator_->mode == Operator::AUTONOMOUS) {
      rclcpp::Rate rate(std::chrono::seconds(5));
      rate.sleep();
    }
    response->success = true;
    response->message = "";
  }

  void onOperator(const tier4_external_api_msgs::msg::Operator::ConstSharedPtr message)
  {
    operator_ = message;
  }

  tier4_external_api_msgs::msg::Operator::ConstSharedPtr operator_;
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::Operator>::SharedPtr sub_operator_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalStartPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
