#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/srv/pause_driving.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class InternalVelocityPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_pause_ = proxy.create_service<tier4_external_api_msgs::srv::PauseDriving>(
      "/api/autoware/set/pause_driving",
      std::bind(&InternalVelocityPlugin::setPauseDriving, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    srv_velocity_ = proxy.create_service<tier4_external_api_msgs::srv::SetVelocityLimit>(
      "/api/autoware/set/velocity_limit",
      std::bind(&InternalVelocityPlugin::setVelocityLimit, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    pub_api_velocity_ = node_ptr->create_publisher<autoware_internal_planning_msgs::msg::VelocityLimit>(
      "/api/autoware/get/velocity_limit", rclcpp::QoS{1}.transient_local());
    pub_planning_velocity_ = node_ptr->create_publisher<autoware_internal_planning_msgs::msg::VelocityLimit>(
      "/planning/scenario_planning/max_velocity_default", rclcpp::QoS{1}.transient_local());
  }

  void on_timer() override
  {
    auto & data = *get_api_data();
    if (data.current_max_velocity) {
      autoware_internal_planning_msgs::msg::VelocityLimit limit = *data.current_max_velocity;
      pub_api_velocity_->publish(limit);
      pub_planning_velocity_->publish(limit);
    }
  }

private:
  void setPauseDriving(
    const tier4_external_api_msgs::srv::PauseDriving::Request::SharedPtr /*request*/,
    const tier4_external_api_msgs::srv::PauseDriving::Response::SharedPtr response)
  {
    response->status = tier4_api_utils::response_success();
  }

  void setVelocityLimit(
    const tier4_external_api_msgs::srv::SetVelocityLimit::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetVelocityLimit::Response::SharedPtr response)
  {
    if (request->velocity > 0.0) {
      autoware_internal_planning_msgs::msg::VelocityLimit limit;
      limit.stamp = node_ptr_->now();
      limit.max_velocity = request->velocity;
      limit.use_constraints = true;
      limit.constraints.min_acceleration = -5.0;
      limit.constraints.max_jerk = 2.0;
      limit.constraints.min_jerk = -2.0;
      pub_planning_velocity_->publish(limit);
      pub_api_velocity_->publish(limit);
    }
    response->status = tier4_api_utils::response_success();
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::PauseDriving>::SharedPtr srv_pause_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetVelocityLimit>::SharedPtr srv_velocity_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr pub_api_velocity_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr pub_planning_velocity_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::InternalVelocityPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
