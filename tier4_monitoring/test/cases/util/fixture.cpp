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

#include "fixture.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_external_api_msgs/msg/response_status.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using tier4_external_api_msgs::msg::ResponseStatus;
using tier4_monitoring::Monitoring;

const std::vector<int64_t> kLevel2Route = {501, 502, 503, 504};
const std::vector<int64_t> kLevel4Route = {502, 503, 504, 505, 506};

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

void MonitoringTest::SetUp()
{
  auto options = rclcpp::NodeOptions();
  options.append_parameter_override("timeout", 1.0);
  options.append_parameter_override("supervisors", std::vector<std::string>{"mot", "remote"});
  options.append_parameter_override("advisors", std::vector<std::string>{"mot", "remote"});
  node_ = std::make_shared<Monitoring>(options);
  mock_ = std::make_shared<MockNode>();
  ASSERT_TRUE(spin_until([this]() { return mock_->is_ready(); }, 10s)) << "discovery timed out";
}

void MonitoringTest::TearDown()
{
  mock_.reset();
  node_.reset();
}

bool MonitoringTest::spin_until(
  const std::function<bool()> & condition, std::chrono::nanoseconds timeout)
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

void MonitoringTest::change_operator(const std::string & name, uint8_t status)
{
  const auto client = mock_->client(name);
  auto future = client->change(status);
  const auto is_received = [&future]() { return future.wait_for(0s) == std::future_status::ready; };
  ASSERT_TRUE(spin_until(is_received, 3s));
  ASSERT_EQ(future.get()->status.code, ResponseStatus::SUCCESS);

  const auto is_changed = [client, status]() {
    return client->status() && client->status()->status == status;
  };
  ASSERT_TRUE(spin_until(is_changed, 3s));
}

void DrivingTest::wait_until_available(uint8_t mode)
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

void DrivingTest::wait_until_route(uint8_t mode)
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

void DrivingTest::expect_not_available(uint8_t mode)
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

void DrivingTest::enable(uint8_t mode)
{
  const auto driving = mock_->driving();
  auto future = driving->enable(mode);
  const auto is_received = [&future]() { return future.wait_for(0s) == std::future_status::ready; };
  ASSERT_TRUE(spin_until(is_received, 3s));

  const auto response = future.get();
  ASSERT_EQ(response->status.code, ResponseStatus::SUCCESS) << response->status.message;

  const auto is_changed = [driving, mode]() {
    return driving->status() && driving->status()->mode == mode;
  };
  ASSERT_TRUE(spin_until(is_changed, 3s));
}

void DrivingTest::enable_error(uint8_t mode, const std::string & message)
{
  auto future = mock_->driving()->enable(mode);
  const auto is_received = [&future]() { return future.wait_for(0s) == std::future_status::ready; };
  ASSERT_TRUE(spin_until(is_received, 3s));

  const auto response = future.get();
  EXPECT_EQ(response->status.code, ResponseStatus::ERROR);
  EXPECT_EQ(response->status.message, message);
}

void DrivingTest::enable_level(uint8_t mode, const std::string & operator_name)
{
  ASSERT_NO_FATAL_FAILURE(change_operator(operator_name, MonitoringStatus::OPERATING));
  ASSERT_NO_FATAL_FAILURE(wait_until_route(mode));
  ASSERT_NO_FATAL_FAILURE(wait_until_available(mode));
  ASSERT_NO_FATAL_FAILURE(enable(mode));
}

void DrivingTest::wait_velocity_limit_set()
{
  const auto planning = mock_->planning();
  const auto is_set = [planning]() { return planning->velocity_limit_set().has_value(); };
  ASSERT_TRUE(spin_until(is_set, 3s)) << "velocity limit is not set";

  const auto & msg = planning->velocity_limit_set().value();
  EXPECT_EQ(msg.max_velocity, 0.0);
  EXPECT_FALSE(msg.use_constraints);
  EXPECT_EQ(msg.sender, "monitoring_api");
  EXPECT_FALSE(planning->velocity_limit_clear().has_value());
}

void DrivingTest::wait_velocity_limit_clear()
{
  const auto planning = mock_->planning();
  const auto is_clear = [planning]() { return planning->velocity_limit_clear().has_value(); };
  ASSERT_TRUE(spin_until(is_clear, 3s)) << "velocity limit is not cleared";

  const auto & msg = planning->velocity_limit_clear().value();
  EXPECT_TRUE(msg.command);
  EXPECT_EQ(msg.sender, "monitoring_api");
  EXPECT_FALSE(planning->velocity_limit_set().has_value());
}

void DrivingTest::expect_no_velocity_limit()
{
  spin_until([]() { return false; }, 500ms);
  EXPECT_FALSE(mock_->planning()->velocity_limit_set().has_value());
  EXPECT_FALSE(mock_->planning()->velocity_limit_clear().has_value());
}
