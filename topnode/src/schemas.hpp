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


constexpr const auto kCpuMemoryUsageSchema =
R"(uint32 pid
CpuUsage cpu_usage
MemoryUsage memory_usage
================================================================================
MSG: topnode_interfaces/msg/CpuUsage
uint64 elapsed_time
uint64 user_mode_time
uint64 total_user_mode_time
uint64 kernel_mode_time
uint64 total_kernel_mode_time
float64 percent
LoadAvg load_average
================================================================================
MSG: topnode_interfaces/msg/LoadAvg
float32 last_1min
float32 last_5min
float32 last_15min
uint32 task_counts
uint32 available_tasks
uint32 last_created_task
================================================================================
MSG: topnode_interfaces/msg/MemoryUsage
uint64 max_resident_set_size
uint64 shared_size
uint64 virtual_size
float64 percent)";

constexpr const auto kMemoryStateSchema =
R"(uint64 total_program_size
uint64 resident_size
uint64 shared_page_count
uint64 text_size
uint64 lib_size
uint64 data_size
uint64 dirty_pages)";

constexpr const auto kIoStatsSchema =
R"(uint64 bytes_read
uint64 bytes_written
uint64 characters_read
uint64 characters_written
uint64 read_syscalls
uint64 write_syscalls
uint64 cancelled_byte_writes)";

constexpr const auto kStatSchema =
R"(uint32 pid
string command
char state
uint32 parent_pid
uint32 group
uint32 session
uint32 tty_number
uint32 tty_gid
uint32 kernel_flags
uint32 minor_faults_count
uint32 children_minor_faults_count
uint32 major_faults_count
uint32 children_major_faults_count
uint32 user_mode_time
uint32 kernel_mode_time
int32 children_user_mode_time
int32 children_kernel_mode_time
int32 priority
int32 niceness
int32 num_threads
int32 deprecated
uint32 start_time
uint32 virtual_mem_size
int32 resident_set_size
uint32 resident_set_limit
uint32 start_of_code
uint32 end_of_code
uint32 start_of_stack
uint32 end_of_stack
uint32 instruction_pointer
uint32 pending_signals
uint32 blocked_signals
uint32 ignored_signals
uint32 caught_signals
uint32 waiting_channel
uint32 swapped_pages
uint32 children_swapped_pages
int32 exit_signal
int32 cpu_number
uint32 real_time_priority
uint32 scheduling_policy
uint32 block_io_delays
uint32 guest_time
int32 children_guest_time
uint32 start_of_data
uint32 end_of_data
uint32 end_of_heap
uint32 start_of_command_line_args
uint32 end_of_command_line_args
uint32 start_of_environment
uint32 end_of_environment
int32 exit_code)";
