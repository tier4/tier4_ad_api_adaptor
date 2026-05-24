#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/converter/response_status.hpp"

#include <array>
#include <autoware/component_interface_specs_universe/localization.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <tier4_external_api_msgs/srv/initialize_pose.hpp>
#include <tier4_external_api_msgs/srv/initialize_pose_auto.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

namespace
{
constexpr double kInitialPoseTimeout = 300.0;
constexpr std::array<double, 36> kParticleCovariance = {
  1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
  0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00,
  0.00, 0.00, 0.00, 10.00};
}  // namespace

class ExternalInitialPosePlugin : public Tier4ApiAdaptorPluginBase
{
  using InitializePose = tier4_external_api_msgs::srv::InitializePose;
  using InitializePoseAuto = tier4_external_api_msgs::srv::InitializePoseAuto;

public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_pose_ = proxy.create_service<InitializePose>(
      "/api/external/set/initialize_pose",
      std::bind(&ExternalInitialPosePlugin::setInitializePose, this, _1, _2),
      rmw_qos_profile_services_default, group_);
    srv_pose_auto_ = proxy.create_service<InitializePoseAuto>(
      "/api/external/set/initialize_pose_auto",
      std::bind(&ExternalInitialPosePlugin::setInitializePoseAuto, this, _1, _2),
      rmw_qos_profile_services_default, group_);

    const autoware::component_interface_utils::NodeAdaptor adaptor(node_ptr);
    adaptor.init_cli(cli_localization_initialize_);
  }

private:
  void setInitializePose(
    const InitializePose::Request::SharedPtr request,
    const InitializePose::Response::SharedPtr response)
  {
    const auto req = std::make_shared<
      autoware::component_interface_specs_universe::localization::Initialize::Service::Request>();
    req->method =
      autoware::component_interface_specs_universe::localization::Initialize::Service::Request::
        AUTO;
    req->pose_with_covariance.push_back(request->pose);
    req->pose_with_covariance.back().pose.covariance = kParticleCovariance;

    try {
      const auto res = cli_localization_initialize_->call(req, kInitialPoseTimeout);
      response->status = external_api::converter::convert(res->status);
    } catch (const autoware::component_interface_utils::ServiceException & error) {
      response->status = tier4_api_utils::response_error(error.what());
    }
  }

  void setInitializePoseAuto(
    const InitializePoseAuto::Request::SharedPtr,
    const InitializePoseAuto::Response::SharedPtr response)
  {
    const auto req = std::make_shared<
      autoware::component_interface_specs_universe::localization::Initialize::Service::Request>();
    req->method =
      autoware::component_interface_specs_universe::localization::Initialize::Service::Request::
        AUTO;

    try {
      const auto res = cli_localization_initialize_->call(req, kInitialPoseTimeout);
      response->status = external_api::converter::convert(res->status);
    } catch (const autoware::component_interface_utils::ServiceException & error) {
      response->status = tier4_api_utils::response_error(error.what());
    }
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<InitializePose>::SharedPtr srv_pose_;
  tier4_api_utils::Service<InitializePoseAuto>::SharedPtr srv_pose_auto_;
  autoware::component_interface_utils::Client<
    autoware::component_interface_specs_universe::localization::Initialize>::SharedPtr
    cli_localization_initialize_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalInitialPosePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
