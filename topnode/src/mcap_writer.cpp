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

#include "topnode/mcap_writer.hpp"
#include <rmw/serialized_message.h>

namespace topnode
{

McapWriter::McapWriter(const std::string &output_filename):
  writer_options_("ros2")
{
  output_file_ = std::ofstream(output_filename, std::ios::binary);
  writer_.open(output_file_, writer_options_);

  serialized_msg_ = rmw_get_zero_initialized_serialized_message();

  auto allocator = rcutils_get_default_allocator();
  auto ret = rmw_serialized_message_init(
      &serialized_msg_,
      100u,
      &allocator
  );

  if (ret != RMW_RET_OK)
  {
    std::cerr << "Failed to initialize serialized message" << std::endl;
  }
}

McapWriter::~McapWriter()
{
  auto ret = rmw_serialized_message_fini(&serialized_msg_);

  if (ret != RMW_RET_OK) {
    std::cerr << "Failed to fini serialized message" << std::endl;
  }
  writer_.close();
}

void McapWriter::add_topic(const std::string &topic_name,
                           const std::string &message_type,
                           const std::string &schema_text)
{
  mcap::Schema schema;
  schema.encoding = "ros2msg";
  schema.name = message_type;
  schema.data.assign(reinterpret_cast<const std::byte *>(schema_text.data()),
                     reinterpret_cast<const std::byte *>(schema_text.data() + schema_text.size()));
  writer_.addSchema(schema);

  schemas_.emplace(schema.id, schema);
  topic_to_schema_.emplace(topic_name, schema.id);

  mcap::Channel channel;
  channel.topic = topic_name;
  channel.messageEncoding = "cdr";
  channel.schemaId = schema.id;

  writer_.addChannel(channel);
  channels_.emplace(channel.id, channel);
  topic_to_channel_.emplace(topic_name, channel.id);
}
}  // namespace topnode
