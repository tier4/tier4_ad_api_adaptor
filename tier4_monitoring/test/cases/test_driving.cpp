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
  std::string operator_name;   // The operator that is responsible for the level.
  std::vector<int64_t> route;  // The lanelets that the route consists of.
};

struct DrivingOperatorTestParam
{
  uint8_t mode;
  std::string operator_name;   // The operator that is responsible for the level.
  std::vector<int64_t> route;  // The lanelets that the route consists of.
  uint8_t status;              // The status of the operator.
  bool available;              // Whether the level becomes available with the status.
};

std::string to_level_name(uint8_t mode)
{
  // clang-format off
  switch (mode) {
    case DrivingStatus::STOP:   return "Stop";
    case DrivingStatus::LEVEL2: return "Level2";
    case DrivingStatus::LEVEL4: return "Level4";
    default:                    return "Unknown";
  }
  // clang-format on
}

std::string to_status_name(uint8_t status)
{
  // clang-format off
  switch (status) {
    case MonitoringStatus::UNAVAILABLE: return "Unavailable";
    case MonitoringStatus::AVAILABLE:   return "Available";
    case MonitoringStatus::OPERATING:   return "Operating";
    default:                            return "Unknown";
  }
  // clang-format on
}

// The message that the enable service returns when the requested level is not available.
std::string to_unavailable_message(uint8_t mode)
{
  if (mode == DrivingStatus::LEVEL2) return "level2 is not available";
  if (mode == DrivingStatus::LEVEL4) return "level4 is not available";
  return "unknown mode";
}

std::string to_level_test_name(const testing::TestParamInfo<DrivingLevelTestParam> & info)
{
  return to_level_name(info.param.mode);
}

std::string to_operator_test_name(const testing::TestParamInfo<DrivingOperatorTestParam> & info)
{
  return to_level_name(info.param.mode) + to_status_name(info.param.status);
}

class DrivingTest : public testing::Test
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

  // Waits until the given level becomes available. The availability is updated by the timer of
  // the node, so it is not reflected immediately after the map, the route and the operator.
  void wait_until_available(uint8_t mode)
  {
    const auto driving = mock_->driving();
    const auto is_available = [driving, mode]() {
      const auto & status = driving->status();
      if (!status) return false;
      if (mode == DrivingStatus::LEVEL2) return status->is_level2_available;
      if (mode == DrivingStatus::LEVEL4) return status->is_level4_available;
      return true;
    };
    ASSERT_TRUE(spin_until(is_available, 3s));
  }

  // Waits until the route of the given level is reflected in the status. This makes sure that the
  // level is not available because of the operator status and not because of the route.
  void wait_until_route(uint8_t mode)
  {
    const auto driving = mock_->driving();
    const auto is_route = [driving, mode]() {
      const auto & status = driving->status();
      if (!status) return false;
      if (mode == DrivingStatus::LEVEL2) return status->is_level2_route;
      if (mode == DrivingStatus::LEVEL4) return status->is_level4_route;
      return true;
    };
    ASSERT_TRUE(spin_until(is_route, 3s));
  }

  // Waits for a while and checks that the given level is neither available nor enabled.
  void expect_not_available(uint8_t mode)
  {
    spin_until([]() { return false; }, 500ms);
    const auto & status = mock_->driving()->status();
    ASSERT_TRUE(status.has_value());
    EXPECT_NE(status->mode, mode);
    if (mode == DrivingStatus::LEVEL2) {
      EXPECT_FALSE(status->is_level2_available);
    }
    if (mode == DrivingStatus::LEVEL4) {
      EXPECT_FALSE(status->is_level4_available);
    }
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

  // Calls the enable service and checks that the request is rejected.
  void enable_error(uint8_t mode, const std::string & message)
  {
    auto future = mock_->driving()->enable(mode);
    const auto is_received = [&future]() {
      return future.wait_for(0s) == std::future_status::ready;
    };
    ASSERT_TRUE(spin_until(is_received, 3s));

    const auto response = future.get();
    EXPECT_EQ(response->status.code, ResponseStatus::ERROR);
    EXPECT_EQ(response->status.message, message);
  }

  std::shared_ptr<Monitoring> node_;
  std::shared_ptr<MockNode> mock_;
  bool heartbeat_enabled_ = true;
};

class DrivingLevelTest : public DrivingTest,
                         public testing::WithParamInterface<DrivingLevelTestParam>
{
};

class DrivingOperatorTest : public DrivingTest,
                            public testing::WithParamInterface<DrivingOperatorTestParam>
{
};

TEST_P(DrivingLevelTest, Enable)
{
  const auto & param = GetParam();
  mock_->routing()->set_route(param.route);
  ASSERT_NO_FATAL_FAILURE(change_operator(param.operator_name, MonitoringStatus::OPERATING));
  ASSERT_NO_FATAL_FAILURE(wait_until_available(param.mode));
  enable(param.mode);
}

// The level2 transition needs a supervisor that is operating, so it is rejected while the
// supervisor is unavailable or available. The level4 transition needs an advisor that is available
// or operating, so it is accepted while the advisor is available.
TEST_P(DrivingOperatorTest, Enable)
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

// The level4 transition is rejected because the goal of the route is not the end of the level4
// section of the start lanelet. The level4 section of the lanelet 502 ends at the lanelet 506.
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

INSTANTIATE_TEST_SUITE_P(
  Monitoring, DrivingLevelTest,
  testing::Values(
    DrivingLevelTestParam{DrivingStatus::LEVEL2, "supervisor/mot", {501, 502, 503, 504}},
    DrivingLevelTestParam{DrivingStatus::LEVEL4, "advisor/mot", {502, 503, 504, 505, 506}}),
  to_level_test_name);

INSTANTIATE_TEST_SUITE_P(
  Monitoring, DrivingOperatorTest,
  testing::Values(
    DrivingOperatorTestParam{
      DrivingStatus::LEVEL2,
      "supervisor/mot",
      {501, 502, 503, 504},
      MonitoringStatus::UNAVAILABLE,
      false},
    DrivingOperatorTestParam{
      DrivingStatus::LEVEL2,
      "supervisor/mot",
      {501, 502, 503, 504},
      MonitoringStatus::AVAILABLE,
      false},
    DrivingOperatorTestParam{
      DrivingStatus::LEVEL4,
      "advisor/mot",
      {502, 503, 504, 505, 506},
      MonitoringStatus::UNAVAILABLE,
      false},
    DrivingOperatorTestParam{
      DrivingStatus::LEVEL4,
      "advisor/mot",
      {502, 503, 504, 505, 506},
      MonitoringStatus::AVAILABLE,
      true}),
  to_operator_test_name);
