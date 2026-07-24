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

#ifndef MESSAGE_HPP_
#define MESSAGE_HPP_

#include "types.hpp"

#include <cstdint>

namespace tier4_monitoring
{

uint8_t to_monitoring_status(const OperatorStatus & mode);
OperatorStatus from_monitoring_status(const uint8_t & mode);

uint8_t to_driving_status(const DrivingLevel & level);
DrivingLevel from_driving_status(const uint8_t & level);

}  // namespace tier4_monitoring

#endif  // MESSAGE_HPP_
