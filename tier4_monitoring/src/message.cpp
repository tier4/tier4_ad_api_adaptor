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

using tier4_external_api_msgs::msg::MonitoringMode;

uint8_t to_msg(const OperatorMode & mode)
{
  // clang-format off
  switch (mode) {
    case OperatorMode::kUnknown:     return MonitoringMode::UNKNOWN;
    case OperatorMode::kTimeout:     return MonitoringMode::TIMEOUT;
    case OperatorMode::kUnavailable: return MonitoringMode::UNAVAILABLE;
    case OperatorMode::kAvailable:   return MonitoringMode::AVAILABLE;
    case OperatorMode::kOperating:   return MonitoringMode::OPERATING;
    default:                         return MonitoringMode::UNKNOWN;
  }
  // clang-format on
}

OperatorMode from_msg(const uint8_t & mode)
{
  // clang-format off
  switch (mode) {
    case MonitoringMode::UNKNOWN:     return OperatorMode::kUnknown;
    case MonitoringMode::TIMEOUT:     return OperatorMode::kTimeout;
    case MonitoringMode::UNAVAILABLE: return OperatorMode::kUnavailable;
    case MonitoringMode::AVAILABLE:   return OperatorMode::kAvailable;
    case MonitoringMode::OPERATING:   return OperatorMode::kOperating;
    default:                          return OperatorMode::kUnknown;
  }
  // clang-format on
}

}  // namespace tier4_monitoring
