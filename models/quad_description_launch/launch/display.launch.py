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
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    declare_robot_cmd = DeclareLaunchArgument(
        'robot', default_value='simple_quad', description='Robot name to start.'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation (Gazebo) clock if true'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to use rviz'
    )

    # Get the directory
    rviz_file = PathJoinSubstitution(
        [FindPackageShare('quad_description_launch'), 'rviz', 'display.rviz']
    )
    xacro_file = get_xacro_path(robot)

    robot_description = Command(['xacro', ' ', xacro_file])

    # Create nodes
    bringup_display_nodes = GroupAction(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'robot_description': robot_description},
                ],
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            # Node(
            #    condition=IfCondition(use_rviz),
            #    package='joint_state_publisher_gui',
            #    executable='joint_state_publisher_gui',
            #    name='joint_state_publisher_gui',
            # ),
            Node(
                condition=IfCondition(use_rviz),
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )

    return LaunchDescription(
        [
            declare_robot_cmd,
            declare_use_sim_time_cmd,
            declare_use_rviz_cmd,
            bringup_display_nodes,
        ]
    )
