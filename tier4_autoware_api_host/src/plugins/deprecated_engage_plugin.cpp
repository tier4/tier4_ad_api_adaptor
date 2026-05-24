#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"
#include "tier4_autoware_api_host/deprecated_api_utils.hpp"

#include <autoware/qos_utils/qos_compatibility.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <tier4_external_api_msgs/msg/engage_status.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

namespace tier4_autoware_api_host::plugin
{

class DeprecatedEngagePlugin : public Tier4ApiAdaptorPluginBase
{
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  using EngageService = tier4_external_api_msgs::srv::Engage;
  using EngageStatus = tier4_external_api_msgs::msg::EngageStatus;

public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    const auto service_qos = AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE();

    autoware_control_change_ = node_ptr->declare_parameter("autoware_control_change", false);
    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_ = node_ptr->create_publisher<EngageStatus>("/api/external/get/engage", 1);
    srv_ = node_ptr->create_service<EngageService>(
      "/api/external/set/engage", std::bind(&DeprecatedEngagePlugin::onEngage, this, _1, _2),
      service_qos, group_);
    cli_change_stop_ = node_ptr->create_client<ChangeOperationMode>(
      "/api/operation_mode/change_to_stop", service_qos, group_);
    cli_change_autonomous_ = node_ptr->create_client<ChangeOperationMode>(
      "/api/operation_mode/change_to_autonomous", service_qos, group_);
    cli_enable_control_ = node_ptr->create_client<ChangeOperationMode>(
      "/api/operation_mode/enable_autoware_control", service_qos, group_);
    cli_disable_control_ = node_ptr->create_client<ChangeOperationMode>(
      "/api/operation_mode/disable_autoware_control", service_qos, group_);

    state_.mode = OperationModeState::UNKNOWN;
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    if (!data.operation_mode_state) {
      return;
    }
    onState(*data.operation_mode_state);
  }

private:
  void onState(const OperationModeState & msg)
  {
    state_ = msg;
    EngageStatus status;
    status.stamp = node_ptr_->now();
    status.engage = msg.mode == OperationModeState::AUTONOMOUS;
    pub_->publish(status);
  }

  void onEngage(
    const EngageService::Request::SharedPtr req, EngageService::Response::SharedPtr res)
  {
    using deprecated_api::is_error;
    using deprecated_api::response_error;
    using deprecated_api::response_ignored;
    using deprecated_api::sync_call;
    using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;

    const auto request = std::make_shared<ChangeOperationMode::Request>();
    const bool is_autonomous_mode = state_.mode == OperationModeState::AUTONOMOUS;
    const bool is_autoware_control = state_.is_autoware_control_enabled;

    if (req->engage && is_autonomous_mode && is_autoware_control) {
      res->status = response_ignored("It is already engaged.");
      return;
    }

    if (autoware_control_change_) {
      const auto client =
        req->engage ? cli_enable_control_ : cli_disable_control_;
      const auto [status, response] = sync_call<ChangeOperationMode>(client, request);
      if (is_error(status)) {
        res->status = status;
        return;
      }
      (void)response;
    }

    const auto client = req->engage ? cli_change_autonomous_ : cli_change_stop_;
    const auto [status, response] = sync_call<ChangeOperationMode>(client, request);
    if (is_error(status)) {
      res->status = status;
      return;
    }
    res->status.code =
      response->status.success ? ResponseStatus::SUCCESS : ResponseStatus::ERROR;
    res->status.message = response->status.message;
  }

  bool autoware_control_change_{false};
  OperationModeState state_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<EngageService>::SharedPtr srv_;
  rclcpp::Publisher<EngageStatus>::SharedPtr pub_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_stop_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_autonomous_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_enable_control_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_disable_control_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::DeprecatedEngagePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
