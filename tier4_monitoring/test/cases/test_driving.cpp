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

#include <cstdint>
#include <string>
#include <vector>

struct DrivingLevelTestParam
{
  uint8_t mode;
  std::string operator_name;   // The operator that is responsible for the level.
  std::vector<int64_t> route;  // The lanelets that the route consists of.
  uint8_t status;              // The status of the operator.
  bool available;              // Whether the level becomes available with the status.
};

// The message that the enable service returns when the requested level is not available.
std::string to_unavailable_message(uint8_t mode)
{
  if (mode == DrivingStatus::LEVEL2) return "level2 is not available";
  if (mode == DrivingStatus::LEVEL4) return "level4 is not available";
  return "unknown mode";
}

class DrivingLevelTest : public DrivingTest,
                         public testing::WithParamInterface<DrivingLevelTestParam>
{
};

// The level2 transition needs a supervisor that is operating, so it is rejected while the
// supervisor is unavailable or available. The level4 transition needs an advisor that is available
// or operating, so it is accepted while the advisor is available.
TEST_P(DrivingLevelTest, Enable)
{
  const auto & param = GetParam();
  mock_->routing()->set_route(param.route);
  ASSERT_NO_FATAL_FAILURE(change_operator(param.operator_name, param.status));
  ASSERT_NO_FATAL_FAILURE(wait_until_route(param.mode));

  if (param.available) {
    ASSERT_NO_FATAL_FAILURE(wait_until_available(param.mode));
    enable(param.mode);
    return;
  }
  ASSERT_NO_FATAL_FAILURE(expect_not_available(param.mode));
  ASSERT_NO_FATAL_FAILURE(enable_error(param.mode, to_unavailable_message(param.mode)));
  ASSERT_NO_FATAL_FAILURE(expect_not_available(param.mode));
}

std::string to_test_name(const testing::TestParamInfo<DrivingLevelTestParam> & info)
{
  return to_level_name(info.param.mode) + to_status_name(info.param.status);
}

// NOLINTBEGIN(whitespace/line_length)
// clang-format off
INSTANTIATE_TEST_SUITE_P(
  Monitoring, DrivingLevelTest,
  testing::Values(
    DrivingLevelTestParam{DrivingStatus::LEVEL2, "supervisor/mot", kLevel2Route, MonitoringStatus::TIMEOUT, false},
    DrivingLevelTestParam{DrivingStatus::LEVEL2, "supervisor/mot", kLevel2Route, MonitoringStatus::UNAVAILABLE, false},
    DrivingLevelTestParam{DrivingStatus::LEVEL2, "supervisor/mot", kLevel2Route, MonitoringStatus::AVAILABLE, false},
    DrivingLevelTestParam{DrivingStatus::LEVEL2, "supervisor/mot", kLevel2Route, MonitoringStatus::OPERATING, true},
    DrivingLevelTestParam{DrivingStatus::LEVEL4, "advisor/mot", kLevel4Route, MonitoringStatus::TIMEOUT, false},
    DrivingLevelTestParam{DrivingStatus::LEVEL4, "advisor/mot", kLevel4Route, MonitoringStatus::UNAVAILABLE, false},
    DrivingLevelTestParam{DrivingStatus::LEVEL4, "advisor/mot", kLevel4Route, MonitoringStatus::AVAILABLE, true},
    DrivingLevelTestParam{DrivingStatus::LEVEL4, "advisor/mot", kLevel4Route, MonitoringStatus::OPERATING, true}
  ),
  to_test_name
);
// clang-format on
// NOLINTEND

TEST_F(DrivingTest, EnableLevel4WithoutLevel4Route)
{
  mock_->routing()->set_route({502, 503, 504, 505});
  ASSERT_NO_FATAL_FAILURE(change_operator("advisor/mot", MonitoringStatus::OPERATING));
  ASSERT_NO_FATAL_FAILURE(
    enable_error(DrivingStatus::LEVEL4, to_unavailable_message(DrivingStatus::LEVEL4)));

  // The mode does not change to level4 because the request is rejected.
  ASSERT_NO_FATAL_FAILURE(expect_not_available(DrivingStatus::LEVEL4));
  EXPECT_FALSE(mock_->driving()->status()->is_level4_route);
}
