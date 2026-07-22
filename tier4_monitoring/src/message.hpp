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

#include <tier4_external_api_msgs/msg/monitoring_mode.hpp>

namespace tier4_monitoring
{

uint8_t to_msg(const OperatorMode & mode);
OperatorMode from_msg(const uint8_t & mode);

}  // namespace tier4_monitoring

#endif  // MESSAGE_HPP_
