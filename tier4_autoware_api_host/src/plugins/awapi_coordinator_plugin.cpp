#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/awapi_info_builder.hpp"

#include <awapi_awiv_adapter/awapi_autoware_util.hpp>
#include <awapi_awiv_adapter/awapi_max_velocity_publisher.hpp>
#include <awapi_awiv_adapter/awapi_velocity_factor_converter.hpp>
#include <awapi_awiv_adapter/awapi_v2x_aggregator.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <tier4_api_msgs/msg/stop_command.hpp>
#include <tier4_api_msgs/msg/velocity_limit.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

namespace tier4_autoware_api_host::plugin
{

class AwapiCoordinatorPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    stop_reason_thresh_dist_ =
      node_ptr->declare_parameter<double>("stop_reason_thresh_dist", 100.0);

    const auto node_max_velocity = node_ptr->declare_parameter<std::string>("node/max_velocity", "");
    const auto param_max_velocity = node_ptr->declare_parameter<std::string>("param/max_velocity", "");
    const double default_max_velocity =
      autoware_api::waitForParam<double>(node_ptr, node_max_velocity, param_max_velocity);

    if (!node_ptr->declare_parameter<bool>("use_control_command_gate", false)) {
      const auto node_emergency_stop =
        node_ptr->declare_parameter<std::string>("node/emergency_stop", "");
      const auto param_emergency_stop =
        node_ptr->declare_parameter<std::string>("param/emergency_stop", "");
      const bool em_stop_param =
        autoware_api::waitForParam<bool>(node_ptr, node_emergency_stop, param_emergency_stop);
      if (!em_stop_param) {
        RCLCPP_WARN_STREAM(
          node_ptr->get_logger(),
          "parameter[check_external_emergency_heartbeat] is false; autoware/put/emergency is not valid");
      }
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    velocity_factor_converter_ = std::make_unique<autoware_api::AutowareIvVelocityFactorConverter>(
      *node_ptr, stop_reason_thresh_dist_);
    v2x_aggregator_ = std::make_unique<autoware_api::AutowareIvV2XAggregator>(*node_ptr);
    max_velocity_publisher_ = std::make_unique<autoware_api::AutowareIvMaxVelocityPublisher>(
      *node_ptr, default_max_velocity);

    pub_v2x_command_ = node_ptr->create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
      "output/v2x_command", 1);
    pub_v2x_state_ = node_ptr->create_publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
      "output/v2x_state", 1);

    sub_velocity_factors_ =
      node_ptr->create_subscription<autoware_adapi_v1_msgs::msg::VelocityFactorArray>(
        "/api/planning/velocity_factors", rclcpp::QoS(100),
        std::bind(&AwapiCoordinatorPlugin::on_velocity_factors, this, std::placeholders::_1));

    sub_v2x_command_ = node_ptr->create_subscription<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
      "/planning/scenario_planning/status/infrastructure_commands", rclcpp::QoS(100),
      std::bind(&AwapiCoordinatorPlugin::on_v2x_command, this, std::placeholders::_1));

    sub_v2x_state_ = node_ptr->create_subscription<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
      "/system/v2x/virtual_traffic_light_states", rclcpp::QoS(100),
      std::bind(&AwapiCoordinatorPlugin::on_v2x_state, this, std::placeholders::_1));

    sub_max_velocity_ = node_ptr->create_subscription<tier4_api_msgs::msg::VelocityLimit>(
      "input/max_velocity", 1,
      std::bind(&AwapiCoordinatorPlugin::on_max_velocity, this, std::placeholders::_1));

    sub_temporary_stop_ = node_ptr->create_subscription<tier4_api_msgs::msg::StopCommand>(
      "input/temporary_stop", 1,
      std::bind(&AwapiCoordinatorPlugin::on_temporary_stop, this, std::placeholders::_1));
  }

  void on_timer() override
  {
    update_current_pose();

    if (aggregated_v2x_command_) {
      pub_v2x_command_->publish(*aggregated_v2x_command_);
    }
    if (aggregated_v2x_state_) {
      pub_v2x_state_->publish(*aggregated_v2x_state_);
    }
  }

private:
  void update_current_pose()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose->header = transform.header;
      pose->pose.position.x = transform.transform.translation.x;
      pose->pose.position.y = transform.transform.translation.y;
      pose->pose.position.z = transform.transform.translation.z;
      pose->pose.orientation = transform.transform.rotation;
      get_api_data()->current_pose = pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_STREAM_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000, "cannot get self pose: " << ex.what());
    }
  }

  void on_velocity_factors(
    const autoware_adapi_v1_msgs::msg::VelocityFactorArray::ConstSharedPtr msg)
  {
    get_api_data()->stop_reason = velocity_factor_converter_->updateStopReasonArray(msg);
  }

  void on_v2x_command(const tier4_v2x_msgs::msg::InfrastructureCommandArray::ConstSharedPtr msg)
  {
    aggregated_v2x_command_ = v2x_aggregator_->updateV2XCommand(msg);
    get_api_data()->v2x_command = aggregated_v2x_command_;
  }

  void on_v2x_state(const tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr msg)
  {
    aggregated_v2x_state_ = v2x_aggregator_->updateV2XState(msg);
    get_api_data()->v2x_state = aggregated_v2x_state_;
  }

  void on_max_velocity(const tier4_api_msgs::msg::VelocityLimit::ConstSharedPtr msg)
  {
    get_api_data()->awapi_max_velocity = msg;
    max_velocity_publisher_->statePublisher(build_awapi_info(*get_api_data()));
  }

  void on_temporary_stop(const tier4_api_msgs::msg::StopCommand::ConstSharedPtr msg)
  {
    auto & data = *get_api_data();
    if (data.temporary_stop && data.temporary_stop->stop == msg->stop) {
      return;
    }
    data.temporary_stop = msg;
    max_velocity_publisher_->statePublisher(build_awapi_info(*get_api_data()));
  }

  double stop_reason_thresh_dist_{100.0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<autoware_api::AutowareIvVelocityFactorConverter> velocity_factor_converter_;
  std::unique_ptr<autoware_api::AutowareIvV2XAggregator> v2x_aggregator_;
  std::unique_ptr<autoware_api::AutowareIvMaxVelocityPublisher> max_velocity_publisher_;

  tier4_v2x_msgs::msg::InfrastructureCommandArray::ConstSharedPtr aggregated_v2x_command_;
  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr aggregated_v2x_state_;

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::VelocityFactorArray>::SharedPtr
    sub_velocity_factors_;
  rclcpp::Subscription<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr sub_v2x_command_;
  rclcpp::Subscription<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr sub_v2x_state_;
  rclcpp::Subscription<tier4_api_msgs::msg::VelocityLimit>::SharedPtr sub_max_velocity_;
  rclcpp::Subscription<tier4_api_msgs::msg::StopCommand>::SharedPtr sub_temporary_stop_;

  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr pub_v2x_command_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr pub_v2x_state_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::AwapiCoordinatorPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
