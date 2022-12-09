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

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <malloc.h>
#include <ranges>
#include <regex>
#include <sstream>
#include <string_view>
#include <sys/resource.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "topnode/resource_info.hpp"
#include "topnode/resource_monitor_node.hpp"

ResourceMonitorNode::ResourceMonitorNode(rclcpp::NodeOptions options)
    : Node("resource_monitor", options) {
  declare_parameter("publish_cpu_memory_usage", true);
  declare_parameter("publish_memory_state", true);
  declare_parameter("publish_io_stats", false);
  declare_parameter("publish_stat", false);
  declare_parameter("publish_period_ms", 1000);

  if (get_parameter("publish_cpu_memory_usage")
          .get_parameter_value()
          .get<bool>()) {
    cpu_memory_usage_publisher_ =
        create_publisher<topnode_interfaces::msg::CpuMemoryUsage>(
            "~/cpu_memory_usage", 10);
  }
  if (get_parameter("publish_memory_state").get_parameter_value().get<bool>()) {
    memory_state_publisher_ =
        create_publisher<topnode_interfaces::msg::MemoryState>("~/memory_state",
                                                               10);
  }
  if (get_parameter("publish_io_stats").get_parameter_value().get<bool>()) {
    io_stats_publisher_ =
        create_publisher<topnode_interfaces::msg::IoStats>("~/io_stats", 10);
  }
  if (get_parameter("publish_stat").get_parameter_value().get<bool>()) {
    stat_publisher_ =
        create_publisher<topnode_interfaces::msg::Stat>("~/stat", 10);
  }

  timer_ = create_wall_timer(
      std::chrono::milliseconds(get_parameter("publish_period_ms")
                                    .get_parameter_value()
                                    .get<int64_t>()),
      std::bind(&ResourceMonitorNode::publish_resource_usage, this));

  last_tick_times_.ticks_per_second = sysconf(_SC_CLK_TCK);
  last_tick_times_.last_measure_time = get_clock()->now();
  last_tick_times_.user_mode_time = 0;
  last_tick_times_.kernel_mode_time = 0;
  last_tick_times_.utime = 0;
  last_tick_times_.stime = 0;
  pid_ = getpid();
}

void ResourceMonitorNode::publish_resource_usage() {
  if (cpu_memory_usage_publisher_) {
    publish_cpu_memory_usage();
  }
  if (memory_state_publisher_) {
    publish_memory_state();
  }
  if (io_stats_publisher_) {
    publish_io_stats();
  }
  if (stat_publisher_) {
    publish_stat();
  }
}

void ResourceMonitorNode::publish_cpu_memory_usage() {
  auto message = topnode_interfaces::msg::CpuMemoryUsage();

  message.cpu_usage =
      resource_info::get_cpu_usage(proc_root_, get_clock(), last_tick_times_);
  message.memory_usage = resource_info::get_memory_usage(proc_root_);

  cpu_memory_usage_publisher_->publish(message);
}

void ResourceMonitorNode::publish_memory_state() {
  memory_state_publisher_->publish(resource_info::get_memory_state(proc_root_));
}

void ResourceMonitorNode::publish_io_stats() {
  io_stats_publisher_->publish(resource_info::get_io_stats(proc_root_));
}

void ResourceMonitorNode::publish_stat() {
  stat_publisher_->publish(resource_info::get_stat(proc_root_));
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ResourceMonitorNode);
