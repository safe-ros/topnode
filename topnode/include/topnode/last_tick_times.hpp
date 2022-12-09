// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef TOPNODE__LAST_TICK_TIMES_HPP_
#define TOPNODE__LAST_TICK_TIMES_HPP_

#include <rclcpp/time.hpp>

#include "topnode/visibility.hpp"

namespace resource_info {

struct LastTickTimes {
  long ticks_per_second = 0;
  rclcpp::Time last_measure_time;
  uint64_t user_mode_time = 0;
  uint64_t kernel_mode_time = 0;
  uint64_t utime = 0;
  uint64_t stime = 0;
};

}; // namespace resource_info

#endif // TOPNODE__LAST_TICK_TIMES_HPP_
