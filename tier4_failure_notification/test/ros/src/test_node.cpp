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

#include "failure_notification.hpp"
#include "mock/mock.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace autoware::failure_notification;  // NOLINT(build/namespaces)

std::filesystem::path resource(const std::string & path)
{
  return std::filesystem::path(TEST_RESOURCE_PATH) / path;
}

DiagNodeStruct create_node_struct(const std::string & path)
{
  DiagNodeStruct msg;
  msg.path = path;
  return msg;
}

DiagNodeStatus create_node_status(uint8_t level, uint8_t input)
{
  DiagNodeStatus msg;
  msg.level = level;
  msg.input_level = input;
  return msg;
}

DiagGraphStruct create_struct()
{
  DiagGraphStruct msg;
  msg.nodes.push_back(create_node_struct("/path/foo"));
  msg.nodes.push_back(create_node_struct("/path/bar"));
  msg.nodes.push_back(create_node_struct("/path/baz"));
  return msg;
}

DiagGraphStatus create_status(const std::vector<std::pair<uint8_t, uint8_t>> & levels)
{
  DiagGraphStatus msg;
  for (const auto & level : levels) {
    msg.nodes.push_back(create_node_status(level.first, level.second));
  }
  return msg;
}

FailureNotificationArray test_main(const std::vector<std::pair<uint8_t, uint8_t>> & levels)
{
  auto options = rclcpp::NodeOptions();
  options.append_parameter_override("error_file", resource("errors.yaml"));
  options.append_parameter_override("interval", 1.0);

  auto node = std::make_shared<FailureNotification>(options);
  auto mock = std::make_shared<MockNode>();

  mock->publish_struct(create_struct());
  mock->publish_status(create_status(levels));

  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node->get_rclcpp_node());
    rclcpp::spin_some(mock);
  }

  EXPECT_EQ(1, mock->notifications.size());
  return mock->notifications[0];
}

TEST(FailureNotification, NoError)
{
  const auto errors = test_main({{0, 0}, {0, 0}, {0, 0}});
  EXPECT_TRUE(errors.notifications.empty());
}

TEST(FailureNotification, OneError1)
{
  const auto errors = test_main({{2, 2}, {0, 0}, {0, 0}});
  EXPECT_EQ(errors.notifications.size(), 1);
  EXPECT_EQ(errors.notifications[0].code, "FOO-000");
}

TEST(FailureNotification, OneError2)
{
  const auto errors = test_main({{0, 0}, {2, 2}, {0, 0}});
  EXPECT_EQ(errors.notifications.size(), 1);
  EXPECT_EQ(errors.notifications[0].code, "BAR-000");
}

TEST(FailureNotification, OneError3)
{
  const auto errors = test_main({{0, 0}, {0, 0}, {2, 2}});
  EXPECT_EQ(errors.notifications.size(), 1);
  EXPECT_EQ(errors.notifications[0].code, "BAZ-000");
}

TEST(FailureNotification, TwoError1)
{
  const auto errors = test_main({{2, 2}, {2, 2}, {0, 0}});
  EXPECT_EQ(errors.notifications.size(), 2);
  EXPECT_EQ(errors.notifications[0].code, "BAR-000");
  EXPECT_EQ(errors.notifications[1].code, "FOO-000");
}

TEST(FailureNotification, TwoError2)
{
  const auto errors = test_main({{0, 0}, {2, 2}, {2, 2}});
  EXPECT_EQ(errors.notifications.size(), 2);
  EXPECT_EQ(errors.notifications[0].code, "BAR-000");
  EXPECT_EQ(errors.notifications[1].code, "BAZ-000");
}

TEST(FailureNotification, AllError)
{
  const auto errors = test_main({{2, 2}, {2, 2}, {2, 2}});
  EXPECT_EQ(errors.notifications.size(), 3);
  EXPECT_EQ(errors.notifications[0].code, "BAR-000");
  EXPECT_EQ(errors.notifications[1].code, "BAZ-000");
  EXPECT_EQ(errors.notifications[2].code, "FOO-000");
}

TEST(FailureNotification, ErrorActive)
{
  const auto errors = test_main({{2, 2}, {0, 0}, {0, 0}});
  EXPECT_EQ(errors.notifications.size(), 1);
  EXPECT_EQ(errors.notifications[0].code, "FOO-000");
  EXPECT_FALSE(errors.notifications[0].is_resolved);
}

TEST(FailureNotification, ErrorResolved)
{
  const auto errors = test_main({{2, 0}, {0, 0}, {0, 0}});
  EXPECT_EQ(errors.notifications.size(), 1);
  EXPECT_EQ(errors.notifications[0].code, "FOO-000");
  EXPECT_TRUE(errors.notifications[0].is_resolved);
}
