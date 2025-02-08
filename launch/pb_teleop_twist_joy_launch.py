# Copyright 2025 Lihan Chen
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Create the launch configuration variables
    joy_config = LaunchConfiguration("joy_config")
    joy_dev = LaunchConfiguration("joy_dev")
    joy_vel = LaunchConfiguration("joy_vel")
    publish_stamped_twist = LaunchConfiguration("publish_stamped_twist")
    config_filepath = LaunchConfiguration("config_filepath")

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Declare the launch arguments
    declare_joy_vel_cmd = DeclareLaunchArgument("joy_vel", default_value="cmd_vel")
    declare_joy_config_cmd = DeclareLaunchArgument("joy_config", default_value="xbox")
    declare_joy_dev_cmd = DeclareLaunchArgument("joy_dev", default_value="0")
    declare_publish_stamped_twist_cmd = DeclareLaunchArgument(
        "publish_stamped_twist", default_value="false"
    )
    declare_config_filepath_cmd = DeclareLaunchArgument(
        "config_filepath",
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    get_package_share_directory("pb_teleop_twist_joy"), "config", ""
                )
            ),
            joy_config,
            TextSubstitution(text=".config.yaml"),
        ],
    )

    # Define the nodes
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "device_id": joy_dev,
                "deadzone": 0.3,
                "autorepeat_rate": 20.0,
            }
        ],
    )

    teleop_twist_joy_node = Node(
        package="pb_teleop_twist_joy",
        executable="pb_teleop_twist_joy_node",
        name="pb_teleop_twist_joy",
        parameters=[
            config_filepath,
            {"publish_stamped_twist": publish_stamped_twist},
        ],
        remappings={("/cmd_vel", joy_vel)},
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_joy_vel_cmd)
    ld.add_action(declare_joy_config_cmd)
    ld.add_action(declare_joy_dev_cmd)
    ld.add_action(declare_publish_stamped_twist_cmd)
    ld.add_action(declare_config_filepath_cmd)

    # Add the nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)

    return ld
