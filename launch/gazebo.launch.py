#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from os.path import join

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_rbot = get_package_share_directory('drone_frame_description')

    # Parse robot description from clean xacro file
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'drone_frame_clean.xacro')
    
    print(f"Processing clean XACRO file: {robot_description_file}")
    
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    print("Clean XACRO processing successful")

    # Start Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Start Ignition Fortress directly
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # Spawn Robot in Gazebo using the correct method for Ignition Fortress
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "drone_frame",
            "-allow_renaming", "false",
            "-z", "1.0",  # Higher spawn height for Ignition Fortress
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0"
        ],            
        output='screen',
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
          'config_file': os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml'),
        }],
        output='screen'
    )      

    # Add a longer delay before spawning to ensure Gazebo is fully initialized
    delayed_spawn = TimerAction(
        period=15.0,  # Increased wait time for Ignition Fortress
        actions=[spawn]
    )

    return LaunchDescription([
        # Nodes and Launches
        gazebo,
        start_gazebo_ros_bridge_cmd,
        robot_state_publisher,
        delayed_spawn,  # Use delayed spawn instead of immediate spawn
    ])
