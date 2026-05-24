#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/awapi_info_builder.hpp"

#include <awapi_awiv_adapter/awapi_lane_change_state_publisher.hpp>
#include <memory>

namespace tier4_autoware_api_host::plugin
{

class AwapiLaneChangePlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    const auto bind = [this](auto && func) {
      return std::bind(func, this, std::placeholders::_1);
    };

    sub_lane_change_available_ =
      node_ptr->create_subscription<tier4_planning_msgs::msg::LaneChangeStatus>(
        "/planning/scenario_planning/lane_driving/lane_change_available", 1,
        bind(&AwapiLaneChangePlugin::on_lane_change_available));
    sub_lane_change_ready_ = node_ptr->create_subscription<tier4_planning_msgs::msg::LaneChangeStatus>(
      "/planning/scenario_planning/lane_driving/lane_change_ready", 1,
      bind(&AwapiLaneChangePlugin::on_lane_change_ready));
    sub_lane_change_candidate_ = node_ptr->create_subscription<autoware_planning_msgs::msg::Path>(
      "/planning/scenario_planning/lane_driving/lane_change_candidate_path", 1,
      bind(&AwapiLaneChangePlugin::on_lane_change_candidate));

    publisher_ = std::make_unique<autoware_api::AutowareIvLaneChangeStatePublisher>(*node_ptr);
  }

  void on_timer() override
  {
    publisher_->statePublisher(build_awapi_info(*get_api_data()));
  }

private:
  void on_lane_change_available(const tier4_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr msg)
  {
    get_api_data()->lane_change_available = msg;
  }

  void on_lane_change_ready(const tier4_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr msg)
  {
    get_api_data()->lane_change_ready = msg;
  }

  void on_lane_change_candidate(const autoware_planning_msgs::msg::Path::ConstSharedPtr msg)
  {
    get_api_data()->lane_change_candidate_path = msg;
  }

  std::unique_ptr<autoware_api::AutowareIvLaneChangeStatePublisher> publisher_;
  rclcpp::Subscription<tier4_planning_msgs::msg::LaneChangeStatus>::SharedPtr
    sub_lane_change_available_;
  rclcpp::Subscription<tier4_planning_msgs::msg::LaneChangeStatus>::SharedPtr sub_lane_change_ready_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr sub_lane_change_candidate_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::AwapiLaneChangePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
