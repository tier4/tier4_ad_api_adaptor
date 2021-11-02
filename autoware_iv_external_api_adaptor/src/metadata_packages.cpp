// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "metadata_packages.hpp"
#include <string>
#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "nlohmann/json.hpp"

namespace external_api
{

MetadataPackages::MetadataPackages(const rclcpp::NodeOptions & options)
: Node("external_api_metadata_packages", options)
{
  using namespace std::placeholders;
  autoware_api_utils::ServiceProxyNodeInterface proxy(this);

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

  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ = proxy.create_service<autoware_external_api_msgs::srv::GetMetadataPackages>(
    "/api/external/get/metadata/packages",
    std::bind(&MetadataPackages::getVersions, this, _1, _2),
    rmw_qos_profile_services_default, group_);
}

void MetadataPackages::getVersions(
  const autoware_external_api_msgs::srv::GetMetadataPackages::Request::SharedPtr,
  const autoware_external_api_msgs::srv::GetMetadataPackages::Response::SharedPtr response)
{
  response->metadata = metadata_;
  response->status = autoware_api_utils::response_success();
}

}  // namespace external_api

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(external_api::MetadataPackages)
