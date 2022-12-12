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

#ifndef TOPNODE__RESOURCE_INFO_HPP_
#define TOPNODE__RESOURCE_INFO_HPP_

#include <filesystem>

#include <rclcpp/clock.hpp>
#include <topnode_interfaces/msg/cpu_usage.hpp>
#include <topnode_interfaces/msg/io_stats.hpp>
#include <topnode_interfaces/msg/load_avg.hpp>
#include <topnode_interfaces/msg/memory_state.hpp>
#include <topnode_interfaces/msg/memory_usage.hpp>
#include <topnode_interfaces/msg/stat.hpp>

#include "topnode/last_tick_times.hpp"

namespace resource_info {

topnode_interfaces::msg::IoStats
get_io_stats(const std::filesystem::path &proc_root);

topnode_interfaces::msg::Stat get_stat(const std::filesystem::path &proc_root);

topnode_interfaces::msg::MemoryState
get_memory_state(const std::filesystem::path &proc_root);

topnode_interfaces::msg::MemoryUsage
get_memory_usage(const std::filesystem::path &proc_root);

topnode_interfaces::msg::LoadAvg
get_load_average(const std::filesystem::path &proc_root);

topnode_interfaces::msg::CpuUsage
get_cpu_usage(const std::filesystem::path &proc_root,
              const rclcpp::Clock::SharedPtr clock,
              LastTickTimes &last_tick_times);

} // namespace resource_info

#endif // TOPNODE__RESOURCE_INFO_HPP_
