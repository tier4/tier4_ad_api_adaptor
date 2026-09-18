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

#include "util/fixture.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

struct VelocityLimitTestParam
{
  uint8_t mode;
  std::string operator_name;   // The operator that is responsible for the level.
  std::vector<int64_t> route;  // The lanelets that the route consists of.
  uint8_t status;              // The status that makes the enabled level unavailable.
};

class VelocityLimitTest : public DrivingTest,
                          public testing::WithParamInterface<VelocityLimitTestParam>
{
};

// The velocity limit is set when the enabled level becomes unavailable because of the operator
// status, and is cleared when the level becomes available again.
TEST_P(VelocityLimitTest, OperatorStatus)
{
  const auto & param = GetParam();
  mock_->routing()->set_route(param.route);
  ASSERT_NO_FATAL_FAILURE(enable_level(param.mode, param.operator_name));

  // The velocity limit is not set while the level is available.
  ASSERT_NO_FATAL_FAILURE(expect_no_velocity_limit());

  // The level becomes unavailable, so the velocity limit is set.
  ASSERT_NO_FATAL_FAILURE(change_operator(param.operator_name, param.status));
  ASSERT_NO_FATAL_FAILURE(wait_velocity_limit_set());

  // The level becomes available again, so the velocity limit is cleared.
  mock_->planning()->reset();
  ASSERT_NO_FATAL_FAILURE(change_operator(param.operator_name, MonitoringStatus::OPERATING));
  ASSERT_NO_FATAL_FAILURE(wait_velocity_limit_clear());
}

// The velocity limit is set when the operator of the enabled level times out, and is cleared when
// the operator comes back.
TEST_F(DrivingTest, VelocityLimitByTimeout)
{
  mock_->routing()->set_route(kLevel2Route);
  ASSERT_NO_FATAL_FAILURE(enable_level(DrivingStatus::LEVEL2, "supervisor/mot"));

  // The operator times out, so the velocity limit is set. The timeout parameter is 1 second.
  heartbeat_enabled_ = false;
  ASSERT_NO_FATAL_FAILURE(wait_velocity_limit_set());

  // The operator comes back, so the velocity limit is cleared. The heartbeat is resumed first
  // because the operator times out again if the status is changed with the stale heartbeat.
  mock_->planning()->reset();
  heartbeat_enabled_ = true;
  spin_until([]() { return false; }, 200ms);
  ASSERT_NO_FATAL_FAILURE(change_operator("supervisor/mot", MonitoringStatus::OPERATING));
  ASSERT_NO_FATAL_FAILURE(wait_velocity_limit_clear());
}

std::string to_test_name(const testing::TestParamInfo<VelocityLimitTestParam> & info)
{
  return to_level_name(info.param.mode) + to_status_name(info.param.status);
}

// NOLINTBEGIN(whitespace/line_length)
// clang-format off
INSTANTIATE_TEST_SUITE_P(
  Monitoring, VelocityLimitTest,
  testing::Values(
    VelocityLimitTestParam{DrivingStatus::LEVEL2, "supervisor/mot", kLevel2Route, MonitoringStatus::AVAILABLE},
    VelocityLimitTestParam{DrivingStatus::LEVEL2, "supervisor/mot", kLevel2Route, MonitoringStatus::UNAVAILABLE},
    VelocityLimitTestParam{DrivingStatus::LEVEL4, "advisor/mot", kLevel4Route, MonitoringStatus::UNAVAILABLE}
  ),
  to_test_name
);
// clang-format on
// NOLINTEND
