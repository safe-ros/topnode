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

#include "topnode/resource_monitor_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <topnode_interfaces/msg/process_resource_usage.hpp>
#include <topnode_interfaces/msg/cpu_usage.hpp>
#include <topnode_interfaces/msg/load_avg.hpp>
#include <topnode_interfaces/msg/memory_usage.hpp>
#include <topnode_interfaces/msg/memory_state.hpp>
#include <topnode_interfaces/msg/stat.hpp>

using namespace std::chrono_literals;

namespace utils {

std::filesystem::path make_proc_root_path() { return "/proc/self"; }

std::string
read_single_line_string_from_file(const std::filesystem::path &path) {
  std::ifstream f(path);
  if (!f) {
    return "";
  }
  std::string line;
  if (!std::getline(f, line)) {
    return "";
  }
  return line;
}

std::string read_multiline_string_from_file(const std::filesystem::path &path) {
  std::ifstream f(path);
  if (!f) {
    return "";
  }
  std::string line;
  if (!std::getline(f, line, f.widen('\0'))) {
    return "";
  }
  return line;
}

std::vector<std::string>
read_multistring_file(const std::filesystem::path &path) {
  std::vector<std::string> strings;
  std::ifstream f(path);
  if (!f) {
    return strings;
  }
  for (std::string line; std::getline(f, line, f.widen('\0'));) {
    strings.push_back(line);
  }

  return strings;
}

std::string read_link(const std::filesystem::path &path) {
  char buffer[1024];
  ssize_t len;

  if ((len = readlink(path.c_str(), buffer, sizeof(buffer) - 1)) != -1) {
    buffer[len] = '\0';
  }

  return std::string(buffer);
}

std::vector<std::string> read_dir_of_links(const std::filesystem::path &path) {
  std::vector<std::string> result;

  for (auto const &dir_entry : std::filesystem::directory_iterator(path)) {
    result.push_back(read_link(dir_entry.path()));
  }

  return result;
}

std::vector<std::string> list_directory(const std::filesystem::path &path) {
  std::vector<std::string> result;

  for (auto const &dir_entry : std::filesystem::directory_iterator(path)) {
    result.push_back(dir_entry.path());
  }

  return result;
}

std::vector<std::string> read_environment(std::filesystem::path proc_root) {
  return read_multistring_file(proc_root / "environ");
}

// topnode_interfaces::msg::IoStats read_io(std::filesystem::path proc_root) {
//   std::regex re("rchar: ([0-9]+)\nwchar: ([0-9]+)\nsyscr: ([0-9]+)\nsyscw: "
//                 "([0-9]+)\nread_bytes: ([0-9]+)\nwrite_bytes: "
//                 "([0-9]+)\ncancelled_write_bytes: ([0-9]+)");
//   std::string io = read_multiline_string_from_file(proc_root / "io");
//   std::smatch match;
//   topnode_interfaces::msg::IoStats result;
//
//   if (std::regex_search(io, match, re)) {
//     result.bytes_read = std::stoi(match[5].str());
//     result.bytes_written = std::stoi(match[6].str());
//     result.characters_read = std::stoi(match[1].str());
//     result.characters_written = std::stoi(match[2].str());
//     result.read_syscalls = std::stoi(match[3].str());
//     result.write_syscalls = std::stoi(match[4].str());
//     result.cancelled_byte_writes = std::stoi(match[7].str());
//   }
//
//   return result;
// }
//
topnode_interfaces::msg::LoadAvg
read_load_average(std::filesystem::path proc_root) {
  std::regex re(
      "([0-9.]+)\\s+([0-9.]+)\\s+([0-9.]+)\\s+(\\d+)/(\\d+)\\s+(\\d+)");
  topnode_interfaces::msg::LoadAvg load_avg;

  std::string load_avg_str =
      read_multiline_string_from_file(proc_root / "loadavg");

  std::smatch match;
  if (std::regex_search(load_avg_str, match, re)) {
    load_avg.last_1min = std::stof(match[1].str());
    load_avg.last_5min = std::stof(match[2].str());
    load_avg.last_15min = std::stof(match[3].str());
    load_avg.task_counts = std::stof(match[4].str());
    load_avg.available_tasks = std::stof(match[5].str());
    load_avg.last_created_task = std::stof(match[6].str());
  }

  return load_avg;
}

topnode_interfaces::msg::Stat
read_process_stat(std::filesystem::path proc_root) {
  std::regex re(
      "(\\d+)\\s+"      // 1: PID
      "(\\S+)\\s+"      // 2: Command
      "(\\w)\\s+"       // 3: State
      "(\\d+)\\s+"      // 4: Parent PID
      "(\\d+)\\s+"      // 5: Group
      "(\\d+)\\s+"      // 6: Session
      "(\\d+)\\s+"      // 7: TTY number
      "(\\d+)\\s+"      // 8: Terminal process group ID
      "(\\d+)\\s+"      // 9: Kernel flags
      "(\\d+)\\s+"      // 10: Minor faults count
      "(\\d+)\\s+"      // 11: Childrens' minor faults count
      "(\\d+)\\s+"      // 12: Major faults count
      "(\\d+)\\s+"      // 13: Childrens' major faults count
      "(\\d+)\\s+"      // 14: User mode time in clock ticks
      "(\\d+)\\s+"      // 15: Kernel mode time in clock ticks
      "([\\d\\-]+)\\s+" // 16: Childrens' user mode time in clock ticks
      "([\\d\\-]+)\\s+" // 17: Childrens' kernel mode time in clock ticks
      "([\\d\\-]+)\\s+" // 18: Priority
      "([\\d\\-]+)\\s+" // 19: Niceness
      "([\\d\\-]+)\\s+" // 20: Number of threads
      "([\\d\\-]+)\\s+" // 21: Number of jiffies before next SIGALRM
                        // (deprecated; will be 0)
      "(\\d+)\\s+"      // 22: Time after system boot the process was started
      "(\\d+)\\s+"      // 23: Virtual memory size
      "([\\d\\-]+)\\s+" // 24: Resident set size
      "(\\d+)\\s+"      // 25: Resident set size limit
      "(\\d+)\\s+"      // 26: Start of code address
      "(\\d+)\\s+"      // 27: End of code address
      "(\\d+)\\s+"      // 28: Start of stack address
      "(\\d+)\\s+"      // 29: Stack pointer value
      "(\\d+)\\s+"      // 30: Instruction pointer value
      "(\\d+)\\s+"      // 31: Pending signals bitmap - obsolete
      "(\\d+)\\s+"      // 32: Blocked signals bitmap - obsolete
      "(\\d+)\\s+"      // 33: Ignored signals bitmap - obsolete
      "(\\d+)\\s+"      // 34: Caught signals bitmap - obsolete
      "(\\d+)\\s+" // 35: Location where the process is waiting in the kernel
      "(\\d+)\\s+" // 36: Number of pages swapped
      "(\\d+)\\s+" // 37: Cumulative number of pages swapped for children
      "([\\d\\-]+)\\s+" // 38: Signal sent to parent when process dies
      "([\\d\\-]+)\\s+" // 39: CPU number last executed on
      "(\\d+)\\s+"      // 40: Real-time scheduling priority
      "(\\d+)\\s+"      // 41: Scheduling policy
      "(\\d+)\\s+"      // 42: Aggregated block I/O delays in clock ticks
      "(\\d+)\\s+"      // 43: Guest time in clock ticks
      "([\\d\\-]+)\\s+" // 44: Guest time of children in clock ticks
      "(\\d+)\\s+"      // 45: Start of program data
      "(\\d+)\\s+"      // 46: End of program data
      "(\\d+)\\s+"      // 47: End of current program heap
      "(\\d+)\\s+"      // 48: Start address of command-line arguments
      "(\\d+)\\s+"      // 49: End address of command-line arguments
      "(\\d+)\\s+"      // 50: Start address of program environment
      "(\\d+)\\s+"      // 51: End address of program environment
  );
  topnode_interfaces::msg::Stat result;

  std::string stat = read_single_line_string_from_file(proc_root / "stat");

  std::smatch match;
  if (std::regex_search(stat, match, re)) {
    result.pid = std::stoul(match[1].str());
    result.command = match[2].str();
    result.state = match[3].str()[0];
    result.parent_pid = std::stoul(match[4].str());
    result.group = std::stoul(match[5].str());
    result.session = std::stoul(match[6].str());
    result.tty_number = std::stoul(match[7].str());
    result.tty_gid = std::stoul(match[8].str());
    result.kernel_flags = std::stoul(match[9].str());
    result.minor_faults_count = std::stoul(match[10].str());
    result.children_minor_faults_count = std::stoul(match[11].str());
    result.major_faults_count = std::stoul(match[12].str());
    result.children_major_faults_count = std::stoul(match[13].str());
    result.user_mode_time = std::stoul(match[14].str());
    result.kernel_mode_time = std::stoul(match[15].str());
    result.children_user_mode_time = std::stoi(match[16].str());
    result.children_kernel_mode_time = std::stoi(match[17].str());
    result.priority = std::stoi(match[18].str());
    result.niceness = std::stoi(match[19].str());
    result.num_threads = std::stoi(match[20].str());
    result.deprecated = std::stoi(match[21].str());
    result.start_time = std::stoul(match[22].str());
    result.virtual_mem_size = std::stoul(match[23].str());
    result.resident_set_size = std::stoi(match[24].str());
    result.resident_set_limit = std::stoul(match[25].str());
    result.start_of_code = std::stoul(match[26].str());
    result.end_of_code = std::stoul(match[27].str());
    result.start_of_stack = std::stoul(match[28].str());
    result.end_of_stack = std::stoul(match[29].str());
    result.instruction_pointer = std::stoul(match[30].str());
    result.pending_signals = std::stoul(match[31].str());
    result.blocked_signals = std::stoul(match[32].str());
    result.ignored_signals = std::stoul(match[33].str());
    result.caught_signals = std::stoul(match[34].str());
    result.waiting_channel = std::stoul(match[35].str());
    result.swapped_pages = std::stoul(match[36].str());
    result.children_swapped_pages = std::stoul(match[37].str());
    result.exit_signal = std::stoi(match[38].str());
    result.cpu_number = std::stoi(match[39].str());
    result.real_time_priority = std::stoul(match[40].str());
    result.scheduling_policy = std::stoul(match[41].str());
    result.block_io_delays = std::stoul(match[42].str());
    result.guest_time = std::stoul(match[43].str());
    result.children_guest_time = std::stoi(match[44].str());
    result.start_of_data = std::stoul(match[45].str());
    result.end_of_data = std::stoul(match[46].str());
    result.end_of_heap = std::stoul(match[47].str());
    result.start_of_command_line_args = std::stoul(match[48].str());
    result.end_of_command_line_args = std::stoul(match[49].str());
    result.start_of_environment = std::stoul(match[50].str());
    result.end_of_environment = std::stoul(match[51].str());
  }
  return result;
}

topnode_interfaces::msg::MemoryState
read_memory_state(std::filesystem::path proc_root) {
  std::regex re("(\\d+)\\s(\\d+)\\s(\\d+)\\s(\\d+)\\s(\\d+)\\s(\\d+)\\s(\\d+)");
  topnode_interfaces::msg::MemoryState result;

  std::string mem_state =
      read_single_line_string_from_file(proc_root / "statm");

  std::smatch match;
  if (std::regex_search(mem_state, match, re)) {
    result.total_program_size = std::stoi(match[1].str());
    result.resident_size = std::stoi(match[2].str());
    result.shared_page_count = std::stoi(match[3].str());
    result.text_size = std::stoi(match[4].str());
    result.lib_size = std::stoi(match[5].str());
    result.data_size = std::stoi(match[6].str());
    result.dirty_pages = std::stoi(match[7].str());
  }

  return result;
}

topnode_interfaces::msg::MemoryUsage
get_memory_usage(std::filesystem::path proc_root) {
  topnode_interfaces::msg::MemoryUsage result;

  struct rusage ru;
  if (getrusage(RUSAGE_SELF, &ru) == 0) {
    result.max_resident_set_size = ru.ru_maxrss;
  } else {
    result.max_resident_set_size = 0;
  }

  auto memory_state = read_memory_state(proc_root);
  result.shared_size =
      (memory_state.shared_page_count * getpagesize()) >> 10;
  result.virtual_size =
      (memory_state.total_program_size * getpagesize()) >> 10;

  std::regex re("MemTotal:\\s+(\\d+)\\s");
  std::smatch match;

  std::string mem_info =
      utils::read_multiline_string_from_file("/proc/meminfo");
  if (std::regex_search(mem_info, match, re)) {
    uint32_t total_memory = std::stoul(match[1].str()) * 1024;

    result.percent = 
      (static_cast<double>(result.max_resident_set_size) / total_memory) * 100.0;
  } else {
    result.percent = 0.0;
  }

  return result;
}

} // namespace utils

ResourceMonitorNode::ResourceMonitorNode(rclcpp::NodeOptions options)
    : Node("resource_monitor", options) {
  resource_usage_publisher_ =
      create_publisher<topnode_interfaces::msg::ProcessResourceUsage>(
          "process_resource_usage", 10);
  timer_ = create_wall_timer(
      1s, std::bind(&ResourceMonitorNode::publish_resource_usage, this));
  last_measure_time_ = get_clock()->now();

  ticks_per_second_ = sysconf(_SC_CLK_TCK);
}

void ResourceMonitorNode::publish_resource_usage() {
  auto message = topnode_interfaces::msg::ProcessResourceUsage();
  auto proc_root = utils::make_proc_root_path();
  message.pid = getpid();

  message.cpu_usage = get_cpu_usage(proc_root);
  message.memory_usage = utils::get_memory_usage(proc_root);

  resource_usage_publisher_->publish(message);
}

topnode_interfaces::msg::CpuUsage
ResourceMonitorNode::get_cpu_usage(const std::filesystem::path &proc_root) {
  topnode_interfaces::msg::CpuUsage result;

  auto now = get_clock()->now();
  result.elapsed_time = (now - last_measure_time_).nanoseconds();

  struct rusage ru;
  uint64_t current_user_mode_time;
  uint64_t current_kernel_mode_time;
  if (getrusage(RUSAGE_SELF, &ru) == 0) {
    current_user_mode_time = ru.ru_utime.tv_sec * 1'000'000'000 + ru.ru_utime.tv_usec * 1'000;
    current_kernel_mode_time = ru.ru_stime.tv_sec * 1'000'000'000 + ru.ru_stime.tv_usec * 1'000;
  } else {
    auto stat = utils::read_process_stat(proc_root);
    current_user_mode_time = stat.user_mode_time * (1'000'000'000 / ticks_per_second_);
    current_kernel_mode_time = stat.kernel_mode_time * (1'000'000'000 / ticks_per_second_);
  }
  result.user_mode_time = current_user_mode_time - last_tick_user_mode_time_;
  result.total_user_mode_time = current_user_mode_time;
  result.kernel_mode_time = current_kernel_mode_time - last_tick_kernel_mode_time_;
  result.total_kernel_mode_time = current_kernel_mode_time;

  result.percent = (static_cast<double>(result.user_mode_time + result.kernel_mode_time) /
      (now - last_measure_time_).nanoseconds()) * 100.0;

  last_tick_user_mode_time_ = current_user_mode_time;
  last_tick_kernel_mode_time_ = current_kernel_mode_time;
  last_measure_time_ = now;

  result.load_average = utils::read_load_average(proc_root);

  return result;
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ResourceMonitorNode);
