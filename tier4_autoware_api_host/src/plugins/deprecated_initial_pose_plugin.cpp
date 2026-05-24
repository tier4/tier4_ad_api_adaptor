#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/deprecated_api_utils.hpp"

#include <array>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <tier4_external_api_msgs/srv/initialize_pose.hpp>

namespace tier4_autoware_api_host::plugin
{

namespace
{
constexpr std::array<double, 36> kParticleCovariance = {
  1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
  0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00,
  0.00, 0.00, 0.00, 10.00};
}  // namespace

class DeprecatedInitialPosePlugin : public Tier4ApiAdaptorPluginBase
{
  using OldService = tier4_external_api_msgs::srv::InitializePose;
  using NewService = autoware_adapi_v1_msgs::srv::InitializeLocalization;

public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    srv_ = node_ptr->create_service<OldService>(
      "/api/external/set/initialize_pose",
      std::bind(&DeprecatedInitialPosePlugin::onService, this, _1, _2));
    cli_ = node_ptr->create_client<NewService>("/api/localization/initialize");
  }

private:
  void onService(
    const OldService::Request::SharedPtr request,
    const OldService::Response::SharedPtr response)
  {
    const auto req = std::make_shared<NewService::Request>();
    req->pose.push_back(request->pose);
    req->pose.back().pose.covariance = kParticleCovariance;

    const auto [status, res] = deprecated_api::sync_call<NewService>(cli_, req);
    if (deprecated_api::is_error(status)) {
      response->status = status;
      return;
    }
    response->status.code =
      res->status.success ? tier4_external_api_msgs::msg::ResponseStatus::SUCCESS
                          : tier4_external_api_msgs::msg::ResponseStatus::ERROR;
    response->status.message = res->status.message;
  }

  rclcpp::Service<OldService>::SharedPtr srv_;
  rclcpp::Client<NewService>::SharedPtr cli_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::DeprecatedInitialPosePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
