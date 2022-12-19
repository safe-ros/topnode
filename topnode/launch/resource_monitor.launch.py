# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""An example container with a resource monitor in it"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a resource monitor container."""
    container = ComposableNodeContainer(
            name='instrumented_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='topnode',
                    plugin='ResourceMonitorNode',
                    name='resource_monitor',
                    parameters=[{
                        "publish_cpu_memory_usage": True,
                        "publish_memory_state": True,
                        "publish_io_stats": True,
                        "publish_stat": True,
                        "publish_period_ms": 500,
                        "record_cpu_memory_usage": True,
                        "record_memory_state": True,
                        "record_io_stats": True,
                        "record_stat": True,
                    }]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
