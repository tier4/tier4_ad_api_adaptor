#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"

#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <nlohmann/json.hpp>
#include <tier4_external_api_msgs/msg/metadata_packages.hpp>
#include <tier4_external_api_msgs/srv/get_metadata_packages.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalMetadataPackagesPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    const auto resources = ament_index_cpp::get_resources("autoware_metadata_packages");
    nlohmann::json json = nlohmann::json::object();
    for (const auto & resource : resources) {
      const std::string & package = resource.first;
      std::string content;
      ament_index_cpp::get_resource("autoware_metadata_packages", package, content);
      json[package] = nlohmann::json::parse(content);
    }
    metadata_.format = "1";
    metadata_.json = json.dump();

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);
    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = proxy.create_service<tier4_external_api_msgs::srv::GetMetadataPackages>(
      "/api/external/get/metadata/packages",
      std::bind(&ExternalMetadataPackagesPlugin::getVersions, this, _1, _2),
      rmw_qos_profile_services_default, group_);
  }

private:
  void getVersions(
    const tier4_external_api_msgs::srv::GetMetadataPackages::Request::SharedPtr,
    const tier4_external_api_msgs::srv::GetMetadataPackages::Response::SharedPtr response)
  {
    response->metadata = metadata_;
    response->status = tier4_api_utils::response_success();
  }

  tier4_external_api_msgs::msg::MetadataPackages metadata_;
  rclcpp::CallbackGroup::SharedPtr group_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::GetMetadataPackages>::SharedPtr srv_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalMetadataPackagesPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
