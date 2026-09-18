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

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

struct ResponsibleTestParam
{
  uint8_t mot;              // The status of the mot operator.
  uint8_t remote;           // The status of the remote operator.
  std::string responsible;  // The operator that is expected to be responsible. Empty means none.
};

std::string to_test_name(const testing::TestParamInfo<ResponsibleTestParam> & info)
{
  return to_status_name(info.param.mot) + to_status_name(info.param.remote);
}

class ResponsibleTest : public MonitoringTest,
                        public testing::WithParamInterface<ResponsibleTestParam>
{
};

// The first operator that is operating in the group becomes responsible, so the mot operator has
// priority over the remote operator. No operator is responsible unless one of them is operating.
TEST_P(ResponsibleTest, Supervisors)
{
  const auto & param = GetParam();
  ASSERT_NO_FATAL_FAILURE(change_operator("supervisor/mot", param.mot));
  ASSERT_NO_FATAL_FAILURE(change_operator("supervisor/remote", param.remote));

  // Wait until both statuses are published because the group updates them at the same time.
  const auto mot = mock_->client("supervisor/mot");
  const auto remote = mock_->client("supervisor/remote");
  const auto is_updated = [mot, remote, param]() {
    if (!mot->status() || !remote->status()) return false;
    return mot->status()->status == param.mot && remote->status()->status == param.remote;
  };
  ASSERT_TRUE(spin_until(is_updated, 3s));

  EXPECT_EQ(mot->status()->responsible, param.responsible == "mot");
  EXPECT_EQ(remote->status()->responsible, param.responsible == "remote");
}

// NOLINTBEGIN(whitespace/line_length)
// clang-format off
INSTANTIATE_TEST_SUITE_P(
  Monitoring, ResponsibleTest,
  testing::Values(
    ResponsibleTestParam{MonitoringStatus::TIMEOUT,     MonitoringStatus::TIMEOUT,     ""},
    ResponsibleTestParam{MonitoringStatus::TIMEOUT,     MonitoringStatus::UNAVAILABLE, ""},
    ResponsibleTestParam{MonitoringStatus::TIMEOUT,     MonitoringStatus::AVAILABLE,   ""},
    ResponsibleTestParam{MonitoringStatus::TIMEOUT,     MonitoringStatus::OPERATING,   "remote"},
    ResponsibleTestParam{MonitoringStatus::UNAVAILABLE, MonitoringStatus::TIMEOUT,     ""},
    ResponsibleTestParam{MonitoringStatus::UNAVAILABLE, MonitoringStatus::UNAVAILABLE, ""},
    ResponsibleTestParam{MonitoringStatus::UNAVAILABLE, MonitoringStatus::AVAILABLE,   ""},
    ResponsibleTestParam{MonitoringStatus::UNAVAILABLE, MonitoringStatus::OPERATING,   "remote"},
    ResponsibleTestParam{MonitoringStatus::AVAILABLE,   MonitoringStatus::TIMEOUT,     ""},
    ResponsibleTestParam{MonitoringStatus::AVAILABLE,   MonitoringStatus::UNAVAILABLE, ""},
    ResponsibleTestParam{MonitoringStatus::AVAILABLE,   MonitoringStatus::AVAILABLE,   ""},
    ResponsibleTestParam{MonitoringStatus::AVAILABLE,   MonitoringStatus::OPERATING,   "remote"},
    ResponsibleTestParam{MonitoringStatus::OPERATING,   MonitoringStatus::TIMEOUT,     "mot"},
    ResponsibleTestParam{MonitoringStatus::OPERATING,   MonitoringStatus::UNAVAILABLE, "mot"},
    ResponsibleTestParam{MonitoringStatus::OPERATING,   MonitoringStatus::AVAILABLE,   "mot"},
    ResponsibleTestParam{MonitoringStatus::OPERATING,   MonitoringStatus::OPERATING,   "mot"}
  ),
  to_test_name
);
// clang-format on
// NOLINTEND
