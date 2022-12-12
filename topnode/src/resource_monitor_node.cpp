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

#include "topnode/mcap_writer.hpp"
#include "topnode/resource_info.hpp"
#include "topnode/resource_monitor_node.hpp"

#include "schemas.hpp"

constexpr auto kCpuMemoryUsageTopic = "~/cpu_memory_usage";
constexpr auto kMemoryStateTopic = "~/memory_state";
constexpr auto kIoStatsTopic = "~/io_stats";
constexpr auto kStatTopic = "~/stat";

ResourceMonitorNode::ResourceMonitorNode(rclcpp::NodeOptions options)
    : Node("resource_monitor", options)
{
  declare_parameter("publish_cpu_memory_usage", true);
  declare_parameter("publish_memory_state", true);
  declare_parameter("publish_io_stats", false);
  declare_parameter("publish_stat", false);
  declare_parameter("publish_period_ms", 1000);

  declare_parameter("record_cpu_memory_usage", true);
  declare_parameter("record_memory_state", true);
  declare_parameter("record_io_stats", false);
  declare_parameter("record_stat", false);


  if (get_parameter("publish_cpu_memory_usage")
          .get_parameter_value()
          .get<bool>()) {
    cpu_memory_usage_publisher_ =
        create_publisher<topnode_interfaces::msg::CpuMemoryUsage>(
            kCpuMemoryUsageTopic, 10);
  }
  if (get_parameter("publish_memory_state").get_parameter_value().get<bool>()) {
    memory_state_publisher_ =
        create_publisher<topnode_interfaces::msg::MemoryState>(kMemoryStateTopic,
                                                               10);

  }
  if (get_parameter("publish_io_stats").get_parameter_value().get<bool>()) {
    io_stats_publisher_ =
        create_publisher<topnode_interfaces::msg::IoStats>(kIoStatsTopic, 10);
  }
  if (get_parameter("publish_stat").get_parameter_value().get<bool>()) {
    stat_publisher_ =
        create_publisher<topnode_interfaces::msg::Stat>(kStatTopic, 10);
  }

  record_cpu_memory_usage_ = get_parameter("record_cpu_memory_usage")
          .get_parameter_value()
          .get<bool>();
  record_memory_state_ = get_parameter("record_memory_state").get_parameter_value().get<bool>();
  record_io_stats_ = get_parameter("record_io_stats").get_parameter_value().get<bool>();
  record_stat_ = get_parameter("record_stat").get_parameter_value().get<bool>();

  if (record_cpu_memory_usage_ || record_memory_state_ || record_io_stats_ | record_stat_)
  {
    writer_ = std::make_unique<topnode::McapWriter>("resource_monitor.mcap");

    if (record_cpu_memory_usage_)
      writer_->add_topic(kCpuMemoryUsageTopic, "topnode_interfaces/msg/CpuMemoryUsage", kCpuMemoryUsageSchema);
    if (record_memory_state_)
      writer_->add_topic(kMemoryStateTopic, "topnode_interfaces/msg/MemoryState", kMemoryStateSchema);
    if (record_io_stats_)
      writer_->add_topic(kIoStatsTopic, "topnode_interfaces/msg/IoStats", kIoStatsSchema);
    if (record_stat_)
      writer_->add_topic(kStatTopic, "topnode_interfaces/msg/Stat", kStatSchema);
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
  if (cpu_memory_usage_publisher_ || record_cpu_memory_usage_) {
    publish_cpu_memory_usage();
  }
  if (memory_state_publisher_ || record_memory_state_) {
    publish_memory_state();
  }
  if (io_stats_publisher_ || record_io_stats_) {
    publish_io_stats();
  }
  if (stat_publisher_ || record_stat_) {
    publish_stat();
  }
}

void ResourceMonitorNode::publish_cpu_memory_usage() {
  auto message = topnode_interfaces::msg::CpuMemoryUsage();

  message.cpu_usage =
      resource_info::get_cpu_usage(proc_root_, get_clock(), last_tick_times_);
  message.memory_usage = resource_info::get_memory_usage(proc_root_);

  if (cpu_memory_usage_publisher_)
    cpu_memory_usage_publisher_->publish(message);

  if (record_cpu_memory_usage_)
    writer_->write(kCpuMemoryUsageTopic, message);
}

void ResourceMonitorNode::publish_memory_state() {
  auto message = resource_info::get_memory_state(proc_root_);

  if (memory_state_publisher_)
    memory_state_publisher_->publish(message);

  if (record_memory_state_)
    writer_->write(kMemoryStateTopic, message);
}

void ResourceMonitorNode::publish_io_stats() {
  auto message = resource_info::get_io_stats(proc_root_);

  if (io_stats_publisher_)
    io_stats_publisher_->publish(message);

  if (record_io_stats_)
    writer_->write(kIoStatsTopic, message);
}

void ResourceMonitorNode::publish_stat() {
  auto message = resource_info::get_stat(proc_root_);

  if (stat_publisher_)
    stat_publisher_->publish(message);

  if (record_stat_)
    writer_->write(kStatTopic, message);
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ResourceMonitorNode);
