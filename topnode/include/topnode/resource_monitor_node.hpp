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

#ifndef TOPNODE__RESOURCE_MONITOR_NODE_HPP_
#define TOPNODE__RESOURCE_MONITOR_NODE_HPP_

#include <filesystem>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <topnode_interfaces/msg/cpu_memory_usage.hpp>
#include <topnode_interfaces/msg/io_stats.hpp>
#include <topnode_interfaces/msg/memory_state.hpp>
#include <topnode_interfaces/msg/stat.hpp>

#include "topnode/last_tick_times.hpp"
#include "topnode/mcap_writer.hpp"
#include "topnode/visibility.hpp"

class ResourceMonitorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// Constructor
  TOPNODE_PUBLIC ResourceMonitorNode(const rclcpp::NodeOptions & options);

  /// Destructor
  TOPNODE_PUBLIC ~ResourceMonitorNode();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Transition callback for state configuring
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state);

  /// Transition callback for state configuring
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state);

  /// Transition callback for state configuring
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);

  /// Transition callback for state configuring
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);

  /// Transition callback for state configuring
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

private:
  void publish_resource_usage();

  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  rclcpp_lifecycle::LifecyclePublisher<topnode_interfaces::msg::CpuMemoryUsage>::SharedPtr
    cpu_memory_usage_publisher_ = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<topnode_interfaces::msg::MemoryState>::SharedPtr
    memory_state_publisher_ = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<topnode_interfaces::msg::IoStats>::SharedPtr
    io_stats_publisher_ = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<topnode_interfaces::msg::Stat>::SharedPtr stat_publisher_ =
    nullptr;

  bool record_cpu_memory_usage_ = false;
  bool record_memory_state_ = false;
  bool record_io_stats_ = false;
  bool record_stat_ = false;

  void publish_cpu_memory_usage();
  void publish_memory_state();
  void publish_io_stats();
  void publish_stat();

  std::filesystem::path proc_root_ = "/proc/self";
  pid_t pid_;
  resource_info::LastTickTimes last_tick_times_;
  std::unique_ptr<topnode::McapWriter> writer_;
};

#endif  // TOPNODE__RESOURCE_MONITOR_NODE_HPP_
