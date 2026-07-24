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

#include <tier4_external_api_msgs/msg/driving_status.hpp>
#include <tier4_external_api_msgs/msg/monitoring_mode.hpp>

namespace tier4_monitoring
{

using tier4_external_api_msgs::msg::DrivingStatus;
using tier4_external_api_msgs::msg::MonitoringMode;

uint8_t to_monitoring_mode(const OperatorMode & mode)
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

OperatorMode from_monitoring_mode(const uint8_t & mode)
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

uint8_t to_driving_status(const DrivingLevel & level)
{
  // clang-format off
  switch (level) {
    case DrivingLevel::kUnknown: return DrivingStatus::UNKNOWN;
    case DrivingLevel::kStop:    return DrivingStatus::STOP;
    case DrivingLevel::kLevel2:  return DrivingStatus::LEVEL2;
    case DrivingLevel::kLevel4:  return DrivingStatus::LEVEL4;
    default:                     return DrivingStatus::UNKNOWN;
  }
  // clang-format on
}

DrivingLevel from_driving_status(const uint8_t & level)
{
  // clang-format off
  switch (level) {
    case DrivingStatus::UNKNOWN: return DrivingLevel::kUnknown;
    case DrivingStatus::STOP:    return DrivingLevel::kStop;
    case DrivingStatus::LEVEL2:  return DrivingLevel::kLevel2;
    case DrivingStatus::LEVEL4:  return DrivingLevel::kLevel4;
    default:                     return DrivingLevel::kUnknown;
  }
  // clang-format on
}

}  // namespace tier4_monitoring
