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

#include <cctype>
#include <chrono>
#include <cstdint>
#include <string>

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

struct MonitoringStatusTestParam
{
  std::string client;
  uint8_t status;
};

std::string to_test_name(const testing::TestParamInfo<MonitoringStatusTestParam> & info)
{
  std::string name;
  bool upper = true;
  for (const auto & c : info.param.client) {
    if (c == '/') {
      upper = true;
      continue;
    }
    name += upper ? static_cast<char>(std::toupper(c)) : c;
    upper = false;
  }
  return name + to_status_name(info.param.status);
}

class MonitoringStatusTest : public MonitoringTest,
                             public testing::WithParamInterface<MonitoringStatusTestParam>
{
};

TEST_P(MonitoringStatusTest, Change)
{
  const auto & param = GetParam();
  change_operator(param.client, param.status);
}

TEST_P(MonitoringStatusTest, Timeout)
{
  const auto & param = GetParam();
  ASSERT_NO_FATAL_FAILURE(change_operator(param.client, param.status));

  // The status returns to timeout when the heartbeat is no longer sent.
  heartbeat_enabled_ = false;
  const auto client = mock_->client(param.client);
  const auto is_timeout = [client]() {
    return client->status() && client->status()->status == MonitoringStatus::TIMEOUT;
  };
  EXPECT_TRUE(spin_until(is_timeout, 3s));
}

INSTANTIATE_TEST_SUITE_P(
  Monitoring, MonitoringStatusTest,
  testing::Values(
    MonitoringStatusTestParam{"supervisor/mot", MonitoringStatus::UNAVAILABLE},
    MonitoringStatusTestParam{"supervisor/mot", MonitoringStatus::AVAILABLE},
    MonitoringStatusTestParam{"supervisor/mot", MonitoringStatus::OPERATING},
    MonitoringStatusTestParam{"supervisor/remote", MonitoringStatus::UNAVAILABLE},
    MonitoringStatusTestParam{"supervisor/remote", MonitoringStatus::AVAILABLE},
    MonitoringStatusTestParam{"supervisor/remote", MonitoringStatus::OPERATING},
    MonitoringStatusTestParam{"advisor/mot", MonitoringStatus::UNAVAILABLE},
    MonitoringStatusTestParam{"advisor/mot", MonitoringStatus::AVAILABLE},
    MonitoringStatusTestParam{"advisor/mot", MonitoringStatus::OPERATING},
    MonitoringStatusTestParam{"advisor/remote", MonitoringStatus::UNAVAILABLE},
    MonitoringStatusTestParam{"advisor/remote", MonitoringStatus::AVAILABLE},
    MonitoringStatusTestParam{"advisor/remote", MonitoringStatus::OPERATING}),
  to_test_name);
