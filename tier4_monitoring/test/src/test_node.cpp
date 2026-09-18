// Copyright 2026 The Autoware Contributors
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
#include "util/mock.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using namespace tier4_monitoring;  // NOLINT(build/namespaces)

void test_main()
{
  auto options = rclcpp::NodeOptions();
  options.append_parameter_override("timeout", 1.0);
  options.append_parameter_override("supervisors", std::vector<std::string>{"mot", "remote"});
  options.append_parameter_override("advisors", std::vector<std::string>{"mot", "remote"});

  auto node = std::make_shared<Monitoring>(options);
  auto mock = std::make_shared<MockNode>();

  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(mock);
  }
}

TEST(MonitoringStatus, Set)
{
  test_main();
}
