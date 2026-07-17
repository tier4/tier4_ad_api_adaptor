// Copyright 2026 TIER IV, Inc.
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

#include "message.hpp"

namespace tier4_monitoring
{

MonitoringMode to_msg(const OperatorState & state)
{
  const auto convert = [](const OperatorState & state) {
    // clang-format off
    switch (state) {
      case OperatorState::kUnknown:     return MonitoringMode::UNKNOWN;
      case OperatorState::kTimeout:     return MonitoringMode::TIMEOUT;
      case OperatorState::kUnavailable: return MonitoringMode::UNAVAILABLE;
      case OperatorState::kAvailable:   return MonitoringMode::AVAILABLE;
      case OperatorState::kOperating:   return MonitoringMode::OPERATING;
      default:                          return MonitoringMode::UNKNOWN;
    }
    // clang-format on
  };
  MonitoringMode msg;
  msg.mode = convert(state);
  return msg;
}

OperatorState from_msg(const MonitoringMode & msg)
{
  const auto convert = [](const MonitoringMode & msg) {
    // clang-format off
    switch (msg.mode) {
      case MonitoringMode::UNKNOWN:     return OperatorState::kUnknown;
      case MonitoringMode::TIMEOUT:     return OperatorState::kTimeout;
      case MonitoringMode::UNAVAILABLE: return OperatorState::kUnavailable;
      case MonitoringMode::AVAILABLE:   return OperatorState::kAvailable;
      case MonitoringMode::OPERATING:   return OperatorState::kOperating;
      default:                          return OperatorState::kUnknown;
    }
    // clang-format on
  };
  return convert(msg);
}

}  // namespace tier4_monitoring
