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

#ifndef TOPNODE__MCAP_WRITER_HPP_
#define TOPNODE__MCAP_WRITER_HPP_

#include <rmw/ret_types.h>
#include <rmw/serialized_message.h>

#include <chrono>
#include <fstream>
#include <string>
#include <unordered_map>

#include <mcap/mcap.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace topnode
{

class McapWriter
{
public:
  /// Constructor
  explicit McapWriter(const std::string & output_filename);

  /// Destructor
  ~McapWriter();

  /// Add a topic to the current recording
  void add_topic(
    const std::string & topic, const std::string & message_type,
    const std::string & schema);

  /// Write a message to a topic
  template<typename T>
  void write(const std::string & topic, const T & msg)
  {
    const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count();

    auto msg_ts = rosidl_typesupport_cpp::get_message_type_support_handle<T>();

    auto ret = rmw_serialize(&msg, msg_ts, &serialized_msg_);

    if (ret != RMW_RET_OK) {
      std::cerr << "Failed to serialize" << std::endl;
      return;
    }

    mcap::Message mcap_msg;
    mcap_msg.channelId = topic_to_channel_[topic];
    mcap_msg.data = reinterpret_cast<std::byte *>(serialized_msg_.buffer);
    mcap_msg.dataSize = serialized_msg_.buffer_length;
    mcap_msg.logTime = timestamp_ns;

    const auto status = writer_.write(mcap_msg);
    if (!status.ok()) {
      std::cerr << "Failed to write message: " << status.message << "\n";
    }
  }

private:
  std::ofstream output_file_;
  mcap::McapWriterOptions writer_options_;
  mcap::McapWriter writer_;

  std::unordered_map<mcap::SchemaId, mcap::Schema> schemas_;
  std::unordered_map<std::string, mcap::SchemaId> topic_to_schema_;

  std::unordered_map<mcap::ChannelId, mcap::Channel> channels_;
  std::unordered_map<std::string, mcap::ChannelId> topic_to_channel_;

  rmw_serialized_message_t serialized_msg_;
};

}  // namespace topnode
#endif  // TOPNODE__MCAP_WRITER_HPP_
