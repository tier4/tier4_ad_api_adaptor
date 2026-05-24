#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_external_api_msgs/msg/velocity_limit.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExtensionVelocityLimitPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    srv_ = node_ptr->create_service<tier4_external_api_msgs::srv::SetVelocityLimit>(
      "/api/external/set/velocity_limit",
      std::bind(&ExtensionVelocityLimitPlugin::onService, this, _1, _2));
    pub_api_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::VelocityLimit>(
      "/api/external/get/velocity_limit", rclcpp::QoS(1).transient_local());
    pub_planning_ = node_ptr->create_publisher<autoware_internal_planning_msgs::msg::VelocityLimit>(
      "/planning/scenario_planning/max_velocity_default", rclcpp::QoS(1).transient_local());
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.current_max_velocity) {
      return;
    }
    tier4_external_api_msgs::msg::VelocityLimit api;
    api.stamp = data.current_max_velocity->stamp;
    api.velocity = data.current_max_velocity->max_velocity;
    pub_api_->publish(api);
  }

private:
  void onService(
    const tier4_external_api_msgs::srv::SetVelocityLimit::Request::SharedPtr req,
    const tier4_external_api_msgs::srv::SetVelocityLimit::Response::SharedPtr res)
  {
    autoware_internal_planning_msgs::msg::VelocityLimit msg;
    msg.stamp = node_ptr_->now();
    msg.max_velocity = req->velocity;
    pub_planning_->publish(msg);

    res->status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
  }

  rclcpp::Service<tier4_external_api_msgs::srv::SetVelocityLimit>::SharedPtr srv_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::VelocityLimit>::SharedPtr pub_api_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr pub_planning_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExtensionVelocityLimitPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
