# pb_teleop_twist_joy

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb_teleop_twist_joy/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb_teleop_twist_joy/actions/workflows/build_and_test.yml/badge.svg)

> [!NOTE]
> This code is currently quite messy, and it is not recommended to develop it again based on this code. The README update is not timely.

## Overview

The purpose of this package is to provide a generic facility for tele-operating Twist-based ROS 2 robots with a standard joystick.
It converts joy messages to velocity commands.

This node provides no rate limiting or autorepeat functionality.
It is expected that you take advantage of the features built into [joy](https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers) for this.

### Executables

The package comes with the `teleop_node` that republishes `sensor_msgs/msg/Joy` messages as scaled `geometry_msgs/msg/Twist` messages.
The message type can be changed to `geometry_msgs/msg/TwistStamped` by the `publish_stamped_twist` parameter.

### Subscribed Topics

- `joy (sensor_msgs/msg/Joy)`
  - Joystick messages to be translated to velocity commands.

### Published Topics

- `cmd_vel (geometry_msgs/msg/Twist or geometry_msgs/msg/TwistStamped)`
  - Command velocity messages arising from Joystick commands.

- `cmd_gimbal_joint (sensor_msgs/msg/JointState)`
  - Command state messages of gimbal joint position arising from Joystick commands.

### Client

- `nav_to_pose_client_ (nav2_msgs/action/NavigateToPose)`
  - Action client for sending navigation goals to the `navigate_to_pose` action server. This client is used to send the robot to a specified pose in the map.

### Parameters

- `require_enable_button (bool, default: true)`
  - Whether to require the enable button for enabling movement.

- `enable_button (int, default: 0)`
  - Joystick button to enable regular-speed movement.

- `enable_turbo_button (int, default: -1)`
  - Joystick button to enable high-speed movement (disabled when -1).

- `axis_chassis.<axis>`
  - Joystick axis to use for linear movement control.
  - `axis_chassis.x (int, default: 5)`
  - `axis_chassis.y (int, default: -1)`
  - `axis_chassis.yaw (int, default: -1)`

- `scale_chassis.<axis>`
  - Scale to apply to joystick linear axis for regular-speed movement.
  - `scale_chassis.x (double, default: 0.5)`
  - `scale_chassis.y (double, default: 0.0)`
  - `scale_chassis.yaw (double, default: 0.0)`

- `scale_chassis_turbo.<axis>`
  - Scale to apply to joystick linear axis for high-speed movement.
  - `scale_chassis_turbo.x (double, default: 1.0)`
  - `scale_chassis_turbo.y (double, default: 0.0)`
  - `scale_chassis_turbo.yaw (double, default: 0.0)`

- `axis_gimbal.<axis>`
  - Joystick axis to use for angular movement control.
  - `axis_gimbal.yaw (int, default: 2)`
  - `axis_gimbal.pitch (int, default: -1)`
  - `axis_gimbal.roll (int, default: -1)`

- `scale_gimbal.<axis>`
  - Scale to apply to joystick angular axis.
  - `scale_gimbal.yaw (double, default: 0.5)`
  - `scale_gimbal.pitch (double, default: 0.0)`
  - `scale_gimbal.roll (double, default: 0.0)`

- `scale_gimbal_turbo.<axis>`
  - Scale to apply to joystick angular axis for high-speed movement.
  - `scale_gimbal_turbo.yaw (double, default: 1.0)`
  - `scale_gimbal_turbo.pitch (double, default: 0.0)`
  - `scale_gimbal_turbo.roll (double, default: 0.0)`

- `inverted_reverse (bool, default: false)`
  - Whether to invert turning left-right while reversing (useful for differential wheeled robots).

- `publish_stamped_twist (bool, default: false)`
  - Whether to publish `geometry_msgs/msg/TwistStamped` for command velocity messages.

- `robot_base_frame (string, default: 'pb_teleop_twist_joy')`
  - Frame name used for the header of TwistStamped messages and lookup transform.

- `control_mode` (string, default: 'manual_control')
  - Options:
  - `manual_control`: Publish speed directly to robot.
  - `auto_control`: Send lookahead goal to navigation2 to control the robot

## Usage

```zsh
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

```zsh
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb_teleop_twist_joy.git
```

```zsh
cd ~/ros_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```zsh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run

A launch file has been provided which has three arguments which can be changed in the terminal or via your own launch file.
To configure the node to match your joystick a config file can be used.
There are several common ones provided in this package (atk3, ps3-holonomic, ps3, xbox, xd3), located here: [config](./config)

PS3 is default, to run for another config (e.g. xbox) use this:

````bash
ros2 launch pb_teleop_twist_joy pb_teleop_twist_joy_launch.py joy_config:='xbox'
````

**Note**: this launch file also launches the `joy` node so do not run it separately.

### Arguments

- `joy_config (string, default: 'xbox')`
  - Config file to use
- `joy_dev (string, default: '0')`
  - Joystick device to use
- `config_filepath (string, default: './config' + LaunchConfig('joy_config') + '.config.yaml')`
  - Path to config files
- `publish_stamped_twist (bool, default: false)`
  - Whether to publish `geometry_msgs/msg/TwistStamped` for command velocity messages.
