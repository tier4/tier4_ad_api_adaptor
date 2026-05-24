#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/calibration_status_array.hpp>
#include <tier4_external_api_msgs/srv/get_accel_brake_map_calibration_data.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalCalibrationStatusPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::CalibrationStatusArray>(
      "/api/external/get/calibration_status", 1);
    cli_ = proxy.create_client<tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData>(
      "/accel_brake_map_calibrator/get_data_service");
    srv_ = proxy.create_service<tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData>(
      "/api/external/get/accel_brake_map_calibrator/data",
      std::bind(&ExternalCalibrationStatusPlugin::getData, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    last_status_ = nullptr;
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.accel_brake_map_status) {
      return;
    }
    tier4_external_api_msgs::msg::CalibrationStatusArray msg;
    msg.stamp = node_ptr_->now();
    msg.status_array.emplace_back(*data.accel_brake_map_status);
    pub_->publish(msg);
  }

private:
  void getData(
    const tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData::Response::SharedPtr
      response)
  {
    const auto [status, resp] = cli_->call(request, std::chrono::seconds(190));
    if (!tier4_api_utils::is_success(status)) {
      return;
    }
    response->graph_image = resp->graph_image;
    response->accel_map = resp->accel_map;
    response->brake_map = resp->brake_map;
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::CalibrationStatusArray>::SharedPtr pub_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData>::SharedPtr
    cli_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::GetAccelBrakeMapCalibrationData>::SharedPtr
    srv_;
  tier4_external_api_msgs::msg::CalibrationStatus::ConstSharedPtr last_status_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalCalibrationStatusPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
