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

#ifndef MONITORING_HPP_
#define MONITORING_HPP_

#include "operator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace tier4_monitoring
{

class Monitoring : public rclcpp::Node
{
public:
  explicit Monitoring(const rclcpp::NodeOptions & options);

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  OperatorGroup supervisors_;
  OperatorGroup advisors_;
};

}  // namespace tier4_monitoring

#endif  // MONITORING_HPP_
