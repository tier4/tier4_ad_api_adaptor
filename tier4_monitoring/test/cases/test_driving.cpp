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

#include <tier4_external_api_msgs/msg/response_status.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace tier4_monitoring;      // NOLINT(build/namespaces)
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using tier4_external_api_msgs::msg::ResponseStatus;

struct DrivingLevelTestParam
{
  uint8_t mode;
  std::string operator_name;  // The operator that is responsible for the level.
};

std::string to_test_name(const testing::TestParamInfo<DrivingLevelTestParam> & info)
{
  // clang-format off
  switch (info.param.mode) {
    case DrivingStatus::STOP:   return "Stop";
    case DrivingStatus::LEVEL2: return "Level2";
    case DrivingStatus::LEVEL4: return "Level4";
    default:                    return "Unknown";
  }
  // clang-format on
}

class DrivingLevelTest : public testing::TestWithParam<DrivingLevelTestParam>
{
protected:
  void SetUp() override
  {
    auto options = rclcpp::NodeOptions();
    options.append_parameter_override("timeout", 1.0);
    options.append_parameter_override("supervisors", std::vector<std::string>{"mot", "remote"});
    options.append_parameter_override("advisors", std::vector<std::string>{"mot", "remote"});
    node_ = std::make_shared<Monitoring>(options);
    mock_ = std::make_shared<MockNode>();
    ASSERT_TRUE(spin_until([this]() { return mock_->is_ready(); }, 10s)) << "discovery timed out";
  }

  void TearDown() override
  {
    mock_.reset();
    node_.reset();
  }

  bool spin_until(const std::function<bool()> & condition, std::chrono::nanoseconds timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (heartbeat_enabled_) mock_->heartbeat();
      rclcpp::spin_some(node_);
      rclcpp::spin_some(mock_);
      if (condition()) return true;
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  }

  // Calls the change service of the operator and waits until the status is published back.
  void change_operator(const std::string & name, uint8_t status)
  {
    const auto client = mock_->client(name);
    auto future = client->change(status);
    const auto is_received = [&future]() {
      return future.wait_for(0s) == std::future_status::ready;
    };
    ASSERT_TRUE(spin_until(is_received, 3s));
    ASSERT_EQ(future.get()->status.code, ResponseStatus::SUCCESS);

    const auto is_changed = [client, status]() {
      return client->status() && client->status()->status == status;
    };
    ASSERT_TRUE(spin_until(is_changed, 3s));
  }

  // Calls the enable service and waits until the status is published back.
  void enable(uint8_t mode)
  {
    const auto driving = mock_->driving();
    auto future = driving->enable(mode);
    const auto is_received = [&future]() {
      return future.wait_for(0s) == std::future_status::ready;
    };
    ASSERT_TRUE(spin_until(is_received, 3s));

    const auto response = future.get();
    ASSERT_EQ(response->status.code, ResponseStatus::SUCCESS) << response->status.message;

    const auto is_changed = [driving, mode]() {
      return driving->status() && driving->status()->mode == mode;
    };
    ASSERT_TRUE(spin_until(is_changed, 3s));
  }

  std::shared_ptr<Monitoring> node_;
  std::shared_ptr<MockNode> mock_;
  bool heartbeat_enabled_ = true;
};

// TODO(isamu-takagi): The level4 case fails until the map and the route are provided by the
// mock node because the level4 route is not available without them.
TEST_P(DrivingLevelTest, Enable)
{
  const auto & param = GetParam();
  ASSERT_NO_FATAL_FAILURE(change_operator(param.operator_name, MonitoringStatus::OPERATING));
  enable(param.mode);
}

INSTANTIATE_TEST_SUITE_P(
  Monitoring, DrivingLevelTest,
  testing::Values(
    DrivingLevelTestParam{DrivingStatus::LEVEL2, "supervisor/mot"},
    DrivingLevelTestParam{DrivingStatus::LEVEL4, "advisor/mot"}),
  to_test_name);
