#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/srv/set_observer.hpp>
#include <tier4_external_api_msgs/srv/set_operator.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalOperatorPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_set_operator_ = proxy.create_service<tier4_external_api_msgs::srv::SetOperator>(
      "/api/external/set/operator",
      std::bind(&ExternalOperatorPlugin::setOperator, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    srv_set_observer_ = proxy.create_service<tier4_external_api_msgs::srv::SetObserver>(
      "/api/external/set/observer",
      std::bind(&ExternalOperatorPlugin::setObserver, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    cli_set_operator_ = proxy.create_client<tier4_external_api_msgs::srv::SetOperator>(
      "/api/autoware/set/operator");
    cli_set_observer_ = proxy.create_client<tier4_external_api_msgs::srv::SetObserver>(
      "/api/autoware/set/observer");
    pub_operator_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::Operator>(
      "/api/external/get/operator", 1);
    pub_observer_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::Observer>(
      "/api/external/get/observer", 1);
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (data.external_operator) {
      pub_operator_->publish(*data.external_operator);
    }
    if (data.external_observer) {
      pub_observer_->publish(*data.external_observer);
    }
  }

private:
  void setOperator(
    const tier4_external_api_msgs::srv::SetOperator::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetOperator::Response::SharedPtr response)
  {
    const auto [status, resp] = cli_set_operator_->call(request);
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  }

  void setObserver(
    const tier4_external_api_msgs::srv::SetObserver::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetObserver::Response::SharedPtr response)
  {
    const auto [status, resp] = cli_set_observer_->call(request);
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->status = resp->status;
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetOperator>::SharedPtr srv_set_operator_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetObserver>::SharedPtr srv_set_observer_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::SetOperator>::SharedPtr cli_set_operator_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::SetObserver>::SharedPtr cli_set_observer_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Operator>::SharedPtr pub_operator_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Observer>::SharedPtr pub_observer_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalOperatorPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
