# TopNode

A resource-monitoring component node for ROS 2.

## Installation

On `humble`:

```
sudo apt install ros-humble-mcap-vendor

cd ~/ros2_ws/src
git clone https://github.com/safe-ros/topnode
cd ~/ros2_ws

colcon build
source ~/ros2_ws/install/setup.sh
```

On `rolling`:

```
sudo apt install ros-rolling-mcap-vendor 

cd ~/ros2_ws/src
git clone https://github.com/safe-ros/topnode
cd ~/ros2_ws

colcon build
source ~/ros2_ws/install/setup.sh
```

## Usage

To see that the workspace was correctly built and installed, the component should appear in the component list

```
$ ros2 component types
topnode
  ResourceMonitorNode
```


### Standalone Node

The standalone resource monitor may be started with

```
ros2 run topnode resource_monitor
```

### Component Node

The primary intended use of the resource monitor is as a [component node](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html).

When the component is added to a running container, it will monitor the resource usage of that container.

#### Instrumenting a running container

To add the resource monitor node to an existing container:

```
ros2 component load $COMPONENT_MANAGER_NODE topnode ResourceMonitorNode
```

#### Instrumenting a container via launch

To add the resource monitor to a container via launch:

```
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
```

### Controlling the monitor via lifecycle

The monitor is implemented as a [managed node](https://design.ros2.org/articles/node_lifecycle.html)

When the node starts, it is in the `unconfigured` state.
In this state, no parameters have been read and the topics/recording have not started.

To move the node to the `configured` state:

```
ros2 lifecycle set /resource_monitor configure
```

In the `configured` state, parameters are read and the requested topics/recording are initialized but not running.

To move the node to the `active` state:

```
ros2 lifecycle set /resource_monitor activate
```

In the `active` state, publication and recording is running

To stop recording, deactivate the node

```
ros2 lifecycle set /resource_monitor deactivate
```

Once deactivated, recording is finalized.

## API

### Parameters

* `publish_period_ms` (int): Statistics sample period
* `publish_cpu_memory_usage` (bool): Enable/disable CPU and memory usage statistics topic 
* `publish_memory_state` (bool): Enable/disable memory state topic
* `publish_io_stats` (bool): Enable/disable IO statistics topic
* `publish_stat` (bool): Enable/disable stat topic 

* `record_file` (string): filename to record to
* `record_cpu_memory_usage` (bool): Enable/disable CPU and memory usage statistics recording
* `record_memory_state` (bool): Enable/disable memory state recording
* `record_io_stats` (bool): Enable/disable IO statistics recording
* `record_stat` (bool): Enable/disable stat recording

### Topics

* `~/cpu_memory_usage`: CPU and memory usage statistics topic
* `~/memory_state`: Memory state topic
* `~/io_stats`: IO stats topic
* `~/stat`: Stat topic


