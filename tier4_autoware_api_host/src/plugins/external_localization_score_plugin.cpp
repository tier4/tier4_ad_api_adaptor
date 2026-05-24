#include "tier4_autoware_api_host/tier4_api_adaptor_plugin_base.hpp"
#include "tier4_autoware_api_host/api_data.hpp"

#include <tier4_external_api_msgs/msg/localization_score.hpp>
#include <tier4_external_api_msgs/msg/localization_score_array.hpp>

namespace tier4_autoware_api_host::plugin
{

class ExternalLocalizationScorePlugin : public Tier4ApiAdaptorPluginBase
{
  using LocalizationScoreArray = tier4_external_api_msgs::msg::LocalizationScoreArray;
  using LocalizationScore = tier4_external_api_msgs::msg::LocalizationScore;

public:
  void initialize(const std::string & name, rclcpp::Node * node_ptr,
                  const std::shared_ptr<ApiData> & api_data) override
  {
    Tier4ApiAdaptorPluginBase::initialize(name, node_ptr, api_data);
    pub_ = node_ptr->create_publisher<LocalizationScoreArray>(
      "/api/external/get/localization_scores", 1);
    score_tp_.name = "transform_probability";
    score_nvtl_.name = "nearest_voxel_transformation_likelihood";
  }

  void on_timer() override
  {
    const auto & data = *get_api_data();
    LocalizationScoreArray msg;

    if (data.transform_probability) {
      score_tp_.value = data.transform_probability->data;
      msg.values.push_back(score_tp_);
    }
    if (data.nearest_voxel_transformation_likelihood) {
      score_nvtl_.value = data.nearest_voxel_transformation_likelihood->data;
      msg.values.push_back(score_nvtl_);
    }

    if (!msg.values.empty()) {
      pub_->publish(msg);
    }
  }

private:
  rclcpp::Publisher<LocalizationScoreArray>::SharedPtr pub_;
  LocalizationScore score_tp_;
  LocalizationScore score_nvtl_;
};

}  // namespace tier4_autoware_api_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_autoware_api_host::plugin::ExternalLocalizationScorePlugin,
  tier4_autoware_api_host::plugin::Tier4ApiAdaptorPluginBase)
