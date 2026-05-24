#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_control_msgs/srv/external_command_select.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/msg/observer.hpp>
#include <tier4_external_api_msgs/msg/operator.hpp>
#include <tier4_external_api_msgs/srv/set_observer.hpp>
#include <tier4_external_api_msgs/srv/set_operator.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class InternalOperatorPlugin : public Tier4ApiAdaptorPluginBase
{
  using ChangeAutowareControl = autoware_system_msgs::srv::ChangeAutowareControl;
  using ExternalCommandSelect = tier4_control_msgs::srv::ExternalCommandSelect;
  using ExternalCommandSelectorMode = tier4_control_msgs::msg::ExternalCommandSelectorMode;
  using GateMode = tier4_control_msgs::msg::GateMode;
  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;

public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    send_engage_in_emergency_ = node_ptr->declare_parameter("send_engage_in_emergency", false);
    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_set_operator_ = proxy.create_service<tier4_external_api_msgs::srv::SetOperator>(
      "/api/autoware/set/operator",
      std::bind(&InternalOperatorPlugin::setOperator, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    srv_set_observer_ = proxy.create_service<tier4_external_api_msgs::srv::SetObserver>(
      "/api/autoware/set/observer",
      std::bind(&InternalOperatorPlugin::setObserver, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    cli_external_select_ = proxy.create_client<ExternalCommandSelect>(
      "/control/external_cmd_selector/select_external_command");
    cli_autoware_control_ = proxy.create_client<ChangeAutowareControl>(
      "/system/operation_mode/change_autoware_control");
    pub_gate_mode_ = node_ptr->create_publisher<GateMode>("/control/gate_mode_cmd", 1);
    pub_operator_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::Operator>(
      "/api/autoware/get/operator", 1);
    pub_observer_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::Observer>(
      "/api/autoware/get/observer", 1);
  }

  void on_timer() override
  {
    publishOperator();
    publishObserver();
  }

private:
  void setOperator(
    const tier4_external_api_msgs::srv::SetOperator::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetOperator::Response::SharedPtr response)
  {
    switch (request->mode.mode) {
      case tier4_external_api_msgs::msg::Operator::DRIVER:
        response->status = setVehicleEngage(false);
        return;
      case tier4_external_api_msgs::msg::Operator::AUTONOMOUS:
        if (
          !send_engage_in_emergency_ && get_api_data()->emergency &&
          get_api_data()->emergency->emergency) {
          response->status = tier4_api_utils::response_error(
            "ignored request because the status is emergency.");
          return;
        }
        setGateMode(GateMode::AUTO);
        response->status = setVehicleEngage(true);
        return;
      case tier4_external_api_msgs::msg::Operator::OBSERVER:
        setGateMode(GateMode::EXTERNAL);
        response->status = setVehicleEngage(true);
        return;
      default:
        response->status = tier4_api_utils::response_error("Invalid parameter.");
        return;
    }
  }

  void setObserver(
    const tier4_external_api_msgs::srv::SetObserver::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetObserver::Response::SharedPtr response)
  {
    switch (request->mode.mode) {
      case tier4_external_api_msgs::msg::Observer::LOCAL:
        response->status = setExternalSelect(ExternalCommandSelectorMode::LOCAL);
        return;
      case tier4_external_api_msgs::msg::Observer::REMOTE:
        response->status = setExternalSelect(ExternalCommandSelectorMode::REMOTE);
        return;
      default:
        response->status = tier4_api_utils::response_error("Invalid parameter.");
        return;
    }
  }

  void publishOperator()
  {
    using OperatorMsg = tier4_external_api_msgs::msg::Operator;
    const auto & data = *get_api_data();
    if (!data.control_mode || !data.gate_mode) {
      return;
    }
    if (data.control_mode->mode == autoware_vehicle_msgs::msg::ControlModeReport::MANUAL) {
      pub_operator_->publish(tier4_external_api_msgs::build<OperatorMsg>().mode(OperatorMsg::DRIVER));
      return;
    }
    switch (data.gate_mode->data) {
      case GateMode::AUTO:
        pub_operator_->publish(
          tier4_external_api_msgs::build<OperatorMsg>().mode(OperatorMsg::AUTONOMOUS));
        return;
      case GateMode::EXTERNAL:
        pub_operator_->publish(
          tier4_external_api_msgs::build<OperatorMsg>().mode(OperatorMsg::OBSERVER));
        return;
    }
    RCLCPP_ERROR(node_ptr_->get_logger(), "Unknown operator.");
  }

  void publishObserver()
  {
    using ObserverMsg = tier4_external_api_msgs::msg::Observer;
    const auto & data = *get_api_data();
    if (!data.external_select) {
      return;
    }
    switch (data.external_select->data) {
      case ExternalCommandSelectorMode::LOCAL:
        pub_observer_->publish(
          tier4_external_api_msgs::build<ObserverMsg>().mode(ObserverMsg::LOCAL));
        return;
      case ExternalCommandSelectorMode::REMOTE:
        pub_observer_->publish(
          tier4_external_api_msgs::build<ObserverMsg>().mode(ObserverMsg::REMOTE));
        return;
    }
    RCLCPP_ERROR(node_ptr_->get_logger(), "Unknown observer.");
  }

  void setGateMode(GateMode::_data_type data)
  {
    pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(data));
  }

  ResponseStatus setVehicleEngage(bool engage)
  {
    const auto req = std::make_shared<ChangeAutowareControl::Request>();
    req->autoware_control = engage;
    const auto [status, resp] = cli_autoware_control_->call(req);
    if (!tier4_api_utils::is_success(status)) {
      return status;
    }
    if (resp->status.success) {
      return tier4_api_utils::response_success(resp->status.message);
    }
    return tier4_api_utils::response_error(resp->status.message);
  }

  ResponseStatus setExternalSelect(ExternalCommandSelectorMode::_data_type data)
  {
    const auto req = std::make_shared<ExternalCommandSelect::Request>();
    req->mode.data = data;
    const auto [status, resp] = cli_external_select_->call(req);
    if (!tier4_api_utils::is_success(status)) {
      return status;
    }
    if (resp->success) {
      return tier4_api_utils::response_success(resp->message);
    }
    return tier4_api_utils::response_error(resp->message);
  }

  bool send_engage_in_emergency_{false};
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetOperator>::SharedPtr srv_set_operator_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::SetObserver>::SharedPtr srv_set_observer_;
  tier4_api_utils::Client<ExternalCommandSelect>::SharedPtr cli_external_select_;
  tier4_api_utils::Client<ChangeAutowareControl>::SharedPtr cli_autoware_control_;
  rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Operator>::SharedPtr pub_operator_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Observer>::SharedPtr pub_observer_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::InternalOperatorPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
