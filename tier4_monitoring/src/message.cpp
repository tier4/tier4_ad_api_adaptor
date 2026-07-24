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
#include <tier4_external_api_msgs/msg/monitoring_status.hpp>

namespace tier4_monitoring
{

using tier4_external_api_msgs::msg::DrivingStatus;
using tier4_external_api_msgs::msg::MonitoringStatus;

uint8_t to_monitoring_status(const OperatorStatus & mode)
{
  // clang-format off
  switch (mode) {
    case OperatorStatus::kUnknown:     return MonitoringStatus::UNKNOWN;
    case OperatorStatus::kTimeout:     return MonitoringStatus::TIMEOUT;
    case OperatorStatus::kUnavailable: return MonitoringStatus::UNAVAILABLE;
    case OperatorStatus::kAvailable:   return MonitoringStatus::AVAILABLE;
    case OperatorStatus::kOperating:   return MonitoringStatus::OPERATING;
    default:                         return MonitoringStatus::UNKNOWN;
  }
  // clang-format on
}

OperatorStatus from_monitoring_status(const uint8_t & mode)
{
  // clang-format off
  switch (mode) {
    case MonitoringStatus::UNKNOWN:     return OperatorStatus::kUnknown;
    case MonitoringStatus::TIMEOUT:     return OperatorStatus::kTimeout;
    case MonitoringStatus::UNAVAILABLE: return OperatorStatus::kUnavailable;
    case MonitoringStatus::AVAILABLE:   return OperatorStatus::kAvailable;
    case MonitoringStatus::OPERATING:   return OperatorStatus::kOperating;
    default:                            return OperatorStatus::kUnknown;
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
