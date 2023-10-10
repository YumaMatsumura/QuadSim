# Copyright 2023 Yuma Matsumura All rights reserved.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_description_pkg_name(robot_name):
    return PythonExpression(['"', robot_name, '_description', '"'])


def get_xacro_file_name(robot_name):
    return PythonExpression(['"', robot_name, '.urdf.xacro', '"'])


def get_xacro_path(robot_name):
    return PathJoinSubstitution(
        [
            FindPackageShare(get_description_pkg_name(robot_name)),
            'urdf',
            get_xacro_file_name(robot_name),
        ]
    )


def generate_launch_description():
    # Create the launch configuration variables
    robot = LaunchConfiguration('robot')
    use_keyboard = LaunchConfiguration('use_keyboard')
    declare_robot_cmd = DeclareLaunchArgument(
        'robot', default_value='simple_quad', description='Robot name to start.'
    )
    declare_use_keyboard_cmd = DeclareLaunchArgument(
        'use_keyboard',
        default_value='False',
        description='Whether to use keyboard to control robot',
    )

    # Get the file directory
    config_file = PathJoinSubstitution(
        [FindPackageShare('simple_gait_controller'), 'params', 'joy_control_params.yaml']
    )
    xacro_file = get_xacro_path(robot)
    robot_description = Command(['xacro', ' ', xacro_file])

    # Create nodes
    load_joy_nodes = GroupAction(
        condition=UnlessCondition(use_keyboard),
        actions=[
            Node(
                package='simple_gait_controller',
                executable='simple_gait_controller',
                name='simple_gait_controller_node',
                parameters=[{'robot_description': robot_description}, config_file],
                output='screen',
            ),
            Node(
                package='joy_linux',
                executable='joy_linux_node',
                parameters=[{'device_name': '/dev/input/js0'}],
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[config_file],
            ),
        ],
    )

    load_keyboard_node = GroupAction(
        condition=IfCondition(use_keyboard),
        actions=[
            Node(
                package='simple_gait_controller',
                executable='simple_gait_controller',
                name='simple_gait_controller_node',
                parameters=[{'robot_description': robot_description}, config_file],
                output='screen',
            ),
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',
            ),
        ],
    )

    return LaunchDescription(
        [declare_robot_cmd, declare_use_keyboard_cmd, load_joy_nodes, load_keyboard_node]
    )
