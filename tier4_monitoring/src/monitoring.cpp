// Copyright 2026 TIER IV, Inc.
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

#include "monitoring.hpp"

#include <memory>

namespace tier4_monitoring
{

Monitoring::Monitoring(const rclcpp::NodeOptions & options) : Node("monitoring", options)
{
  supervisors_.push_back(std::make_unique<Operator>(*this, "supervisor/driver"));
  supervisors_.push_back(std::make_unique<Operator>(*this, "supervisor/mot"));
  supervisors_.push_back(std::make_unique<Operator>(*this, "supervisor/fms"));
  advisors_.push_back(std::make_unique<Operator>(*this, "advisor/mot"));
  advisors_.push_back(std::make_unique<Operator>(*this, "advisor/fms"));
}

}  // namespace tier4_monitoring

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_monitoring::Monitoring)
