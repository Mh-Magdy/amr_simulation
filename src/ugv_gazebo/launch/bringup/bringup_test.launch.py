#!/usr/bin/env python3
#
# bringup_test.launch.py
# Fully working ROS 2 launch for UGV in Gazebo (Humle / ROS 2)
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------
    # Package directories
    # -----------------------------
    pkg_ugv = get_package_share_directory('ugv_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    launch_file_dir = os.path.join(pkg_ugv, 'launch', 'bringup')
    world_file = os.path.join(pkg_ugv, 'worlds', 'myworld3.world')
    robot_model_file = os.path.join(pkg_ugv, 'models', 'ugv_rover', 'model.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # -----------------------------
    # Gazebo server
    # -----------------------------
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # -----------------------------
    # Gazebo client
    # -----------------------------
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # -----------------------------
    # Spawn UGV after Gazebo starts (TimerAction)
    # -----------------------------
    spawn_ugv_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ugv_rover', '-file', robot_model_file],
        output='screen'
    )

    spawn_ugv_delayed = TimerAction(
        period=5.0,  # delay in seconds to allow Gazebo to start
        actions=[spawn_ugv_cmd]
    )

    # -----------------------------
    # Build launch description
    # -----------------------------
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_ugv_delayed)

    return ld
