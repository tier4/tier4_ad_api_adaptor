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
#include <vector>

namespace tier4_monitoring
{

Monitoring::Monitoring(const rclcpp::NodeOptions & options)
: Node("monitoring", options), driving_(*this), supervisors_("supervisor"), advisors_("advisor")
{
  Operator::timeout = declare_parameter<double>("timeout");

  supervisors_.create(*this, "driver");
  supervisors_.create(*this, "mot");
  supervisors_.create(*this, "fms");
  advisors_.create(*this, "mot");
  advisors_.create(*this, "fms");

  const auto period = rclcpp::Rate(10.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void Monitoring::on_timer()
{
  const auto stamp = now();
  supervisors_.update(stamp);
  supervisors_.publish(stamp);
  advisors_.update(stamp);
  advisors_.publish(stamp);

  const bool level2 = supervisors_.has_responsible();
  const bool level4 = advisors_.has_available();
  driving_.update_available_levels(level2, level4);
  driving_.update(stamp);
  driving_.publish(stamp);
}

}  // namespace tier4_monitoring

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_monitoring::Monitoring)
