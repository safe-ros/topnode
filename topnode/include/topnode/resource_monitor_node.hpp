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

#ifndef TOPNODE__TOPNODE_HPP
#define TOPNODE__TOPNODE_HPP

#include "topnode/visibility.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <topnode_interfaces/msg/process_resource_usage.hpp>

class ResourceMonitorNode : public rclcpp::Node {
public:
  TOPNODE_PUBLIC ResourceMonitorNode(rclcpp::NodeOptions options);

private:
  void publish_resource_usage();

  rclcpp::Publisher<topnode_interfaces::msg::ProcessResourceUsage>::SharedPtr
      resource_usage_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_measure_time_;
  uint64_t last_tick_user_mode_time_ = 0;
  uint64_t last_tick_kernel_mode_time_ = 0;

  void calculate_cpu_percentage(
      topnode_interfaces::msg::ProcessResourceUsage &message);
  void calculate_memory_percentage(
      topnode_interfaces::msg::ProcessResourceUsage &message);
};

#endif // TOPNODE__TOPNODE_HPP
