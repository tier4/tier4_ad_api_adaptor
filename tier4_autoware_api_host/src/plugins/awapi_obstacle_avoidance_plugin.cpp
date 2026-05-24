#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/awapi_info_builder.hpp"

#include <awapi_awiv_adapter/awapi_obstacle_avoidance_state_publisher.hpp>
#include <memory>

namespace tier4_autoware_api_host::plugin
{

class AwapiObstacleAvoidancePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    const auto durable_qos = rclcpp::QoS{1}.transient_local();
    const auto bind = [this](auto && func) {
      return std::bind(func, this, std::placeholders::_1);
    };

    sub_obstacle_avoid_ready_ =
      node_ptr->create_subscription<tier4_planning_msgs::msg::IsAvoidancePossible>(
        "/planning/scenario_planning/lane_driving/obstacle_avoidance_ready", durable_qos,
        bind(&AwapiObstacleAvoidancePlugin::on_obstacle_avoid_ready));
    sub_obstacle_avoid_candidate_ =
      node_ptr->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        "/planning/scenario_planning/lane_driving/obstacle_avoidance_candidate_trajectory", durable_qos,
        bind(&AwapiObstacleAvoidancePlugin::on_obstacle_avoid_candidate));

    publisher_ = std::make_unique<autoware_api::AutowareIvObstacleAvoidanceStatePublisher>(*node_ptr);
  }

  void on_timer() override
  {
    publisher_->statePublisher(build_awapi_info(*get_api_data()));
  }

private:
  void on_obstacle_avoid_ready(const tier4_planning_msgs::msg::IsAvoidancePossible::ConstSharedPtr msg)
  {
    get_api_data()->obstacle_avoid_ready = msg;
  }

  void on_obstacle_avoid_candidate(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
  {
    get_api_data()->obstacle_avoid_candidate_path = msg;
  }

  std::unique_ptr<autoware_api::AutowareIvObstacleAvoidanceStatePublisher> publisher_;
  rclcpp::Subscription<tier4_planning_msgs::msg::IsAvoidancePossible>::SharedPtr
    sub_obstacle_avoid_ready_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_obstacle_avoid_candidate_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::AwapiObstacleAvoidancePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
