#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    
    # Package directories
    pkg_drone = get_package_share_directory('drone_frame_description')
    pkg_auv_control = get_package_share_directory('auv_control')
    pkg_thruster_manager = get_package_share_directory('thruster_manager')
    
    # Configuration files
    thruster_config = os.path.join(pkg_drone, 'config', 'drone_thruster_config.yaml')
    pid_config = os.path.join(pkg_drone, 'config', 'drone_pid_config.yaml')
    slider_config = os.path.join(pkg_drone, 'config', 'drone_slider_control.yaml')
    
    # Parse robot description
    robot_description_file = os.path.join(pkg_drone, 'urdf', 'drone_frame.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone',
        description='Namespace for drone nodes'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Thruster Manager Node
    thruster_manager = Node(
        package='thruster_manager',
        executable='thruster_manager_node',
        name='thruster_manager',
        namespace=namespace,
        parameters=[
            thruster_config,
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        remappings=[
            ('wrench', 'cmd_wrench'),
            ('cmd_thrust', 'thrust_commands')
        ]
    )
    
    # Cascaded PID Controller
    cascaded_pid = Node(
        package='auv_control',
        executable='cascaded_pid',
        name='cascaded_pid_controller',
        namespace=namespace,
        parameters=[
            pid_config,
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        remappings=[
            ('cmd_wrench', 'cmd_wrench'),
            ('cmd_pose', 'cmd_pose'),
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom')
        ]
    )
    
    # Slider Publisher for Manual Control
    slider_publisher = Node(
        package='slider_publisher',
        executable='slider_publisher',
        name='drone_manual_control',
        namespace=namespace,
        parameters=[
            {'config': slider_config},
            {'rate': 10.0},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Thruster Command Mapper (converts thrust commands to motor velocities)
    thruster_mapper = Node(
        package='drone_frame_description',
        executable='thruster_command_mapper.py',
        name='thruster_command_mapper',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Add delays to ensure proper startup sequence
    delayed_thruster_manager = TimerAction(
        period=2.0,
        actions=[thruster_manager]
    )
    
    delayed_pid_controller = TimerAction(
        period=4.0,
        actions=[cascaded_pid]
    )
    
    delayed_slider = TimerAction(
        period=6.0,
        actions=[slider_publisher]
    )
    
    delayed_mapper = TimerAction(
        period=3.0,
        actions=[thruster_mapper]
    )
    
    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        use_sim_time_arg,
        
        # Core nodes
        robot_state_publisher,
        delayed_thruster_manager,
        delayed_pid_controller,
        delayed_slider,
        delayed_mapper,
    ])
