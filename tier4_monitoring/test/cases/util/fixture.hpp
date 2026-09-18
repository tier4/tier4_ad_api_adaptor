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

#ifndef CASES__UTIL__FIXTURE_HPP_
#define CASES__UTIL__FIXTURE_HPP_

#include "mock.hpp"
#include "monitoring.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// The routes used by the tests. Any route is available for level2. The level4 section of the
// lanelet 502 ends at the lanelet 506, so the goal of the level4 route is the lanelet 506.
extern const std::vector<int64_t> kLevel2Route;
extern const std::vector<int64_t> kLevel4Route;

// Converts the enum value to the string used for the test name.
std::string to_level_name(uint8_t mode);
std::string to_status_name(uint8_t status);

// The fixture that runs the monitoring node together with the mock node. The helpers that call a
// service or wait for a message use the gtest assertions, so call them with
// ASSERT_NO_FATAL_FAILURE.
class MonitoringTest : public testing::Test
{
protected:
  void SetUp() override;
  void TearDown() override;

  bool spin_until(const std::function<bool()> & condition, std::chrono::nanoseconds timeout);

  // Calls the change service of the operator and waits until the status is published back.
  void change_operator(const std::string & name, uint8_t status);

  std::shared_ptr<tier4_monitoring::Monitoring> node_;
  std::shared_ptr<MockNode> mock_;
  bool heartbeat_enabled_ = true;
};

// The fixture that adds the helpers for the driving status and the velocity limit.
class DrivingTest : public MonitoringTest
{
protected:
  // Waits until the given level becomes available. The availability is updated by the timer of
  // the node, so it is not reflected immediately after the map, the route and the operator.
  void wait_until_available(uint8_t mode);

  // Waits until the route of the given level is reflected in the status. This makes sure that the
  // level is not available because of the operator status and not because of the route.
  void wait_until_route(uint8_t mode);

  // Waits for a while and checks that the given level is neither available nor enabled.
  void expect_not_available(uint8_t mode);

  // Calls the enable service and waits until the status is published back.
  void enable(uint8_t mode);

  // Calls the enable service and checks that the request is rejected.
  void enable_error(uint8_t mode, const std::string & message);

  // Enables the given level after making the route and the operator satisfy its condition.
  void enable_level(uint8_t mode, const std::string & operator_name);

  // Waits until the velocity limit is set and checks the contents of the request.
  void wait_velocity_limit_set();

  // Waits until the velocity limit is cleared and checks the contents of the request.
  void wait_velocity_limit_clear();

  // Waits for a while and checks that neither request is sent.
  void expect_no_velocity_limit();
};

#endif  // CASES__UTIL__FIXTURE_HPP_
