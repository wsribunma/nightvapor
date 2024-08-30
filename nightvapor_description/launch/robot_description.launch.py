# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
]



def generate_launch_description():
    pkg_nightvapor_description = Path(get_package_share_directory('nightvapor_description'))
    urdf_file = pkg_nightvapor_description / 'urdf'/ 'nightvapor.urdf'
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    nightvapor_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='nightvapor_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_desc},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription(ARGUMENTS + [
        nightvapor_state_publisher
    ])
