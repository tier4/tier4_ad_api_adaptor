#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/map_hash.hpp>
#include <tier4_external_api_msgs/srv/get_text_file.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalMapPlugin : public Tier4ApiAdaptorPluginBase
{
public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);

    using namespace std::placeholders;
    tier4_api_utils::ServiceProxyNodeInterface proxy(node_ptr);

    group_ = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_ = node_ptr->create_publisher<tier4_external_api_msgs::msg::MapHash>(
      "/api/external/get/map/info/hash", rclcpp::QoS{1}.transient_local());
    cli_ = proxy.create_client<tier4_external_api_msgs::srv::GetTextFile>(
      "/api/autoware/get/map/lanelet/xml");
    srv_ = proxy.create_service<tier4_external_api_msgs::srv::GetTextFile>(
      "/api/external/get/map/lanelet/xml",
      std::bind(&ExternalMapPlugin::getLaneletXml, this, _1, _2),
      rmw_qos_profile_services_default, group_);
  }

  void on_timer() override
  {
    if (get_api_data()->map_hash) {
      pub_->publish(*get_api_data()->map_hash);
    }
  }

private:
  void getLaneletXml(
    const tier4_external_api_msgs::srv::GetTextFile::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::GetTextFile::Response::SharedPtr response)
  {
    const auto [status, resp] = cli_->call(request);
    if (!tier4_api_utils::is_success(status)) {
      response->status = status;
      return;
    }
    response->file = resp->file;
    response->status = resp->status;
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::MapHash>::SharedPtr pub_;
  tier4_api_utils::Client<tier4_external_api_msgs::srv::GetTextFile>::SharedPtr cli_;
  tier4_api_utils::Service<tier4_external_api_msgs::srv::GetTextFile>::SharedPtr srv_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalMapPlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
