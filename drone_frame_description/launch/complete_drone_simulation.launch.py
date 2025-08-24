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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Configuration files
    thruster_config = os.path.join(pkg_drone, 'config', 'drone_thruster_config.yaml')
    pid_config = os.path.join(pkg_drone, 'config', 'drone_pid_config.yaml')
    slider_config = os.path.join(pkg_drone, 'config', 'drone_slider_control.yaml')
    ros_gz_bridge_config = os.path.join(pkg_drone, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    # Parse robot description
    robot_description_file = os.path.join(pkg_drone, 'urdf', 'drone_frame.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Launch arguments
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
    
    # 1. Start Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": '-r -v 4 empty.sdf'
        }.items()
    )
    
    # 2. Robot State Publisher
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
    
    # 3. Spawn Drone in Gazebo
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "drone_frame",
            "-allow_renaming", "true",
            "-z", "5.0",
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0",
            "-P", "0.0",
            "-R", "1.5708"
        ],
        output='screen',
    )
    
    # 4. ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config,
        }],
        output='screen'
    )
    
    # 5. Drone Odometry Publisher
    odometry_publisher = Node(
        package='drone_frame_description',
        executable='drone_odometry_publisher.py',
        name='drone_odometry_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 6. Thruster Manager
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
    
    # 7. Thruster Command Mapper
    thruster_mapper = Node(
        package='drone_frame_description',
        executable='thruster_command_mapper.py',
        name='thruster_command_mapper',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 8. Cascaded PID Controller
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
    
    # 9. Slider Publisher for Manual Control
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
    
    # 10. Emergency Stop Script
    emergency_stop = Node(
        package='drone_frame_description',
        executable='stop_all_motors.py',
        name='emergency_stop',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Timing sequence for proper startup
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_drone])
    delayed_bridge = TimerAction(period=3.0, actions=[ros_gz_bridge])
    delayed_odometry = TimerAction(period=8.0, actions=[odometry_publisher])
    delayed_thruster_manager = TimerAction(period=10.0, actions=[thruster_manager])
    delayed_mapper = TimerAction(period=12.0, actions=[thruster_mapper])
    delayed_pid = TimerAction(period=14.0, actions=[cascaded_pid])
    delayed_slider = TimerAction(period=16.0, actions=[slider_publisher])
    
    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        use_sim_time_arg,
        
        # Core simulation
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        delayed_bridge,
        
        # Control system
        delayed_odometry,
        delayed_thruster_manager,
        delayed_mapper,
        delayed_pid,
        delayed_slider,
        
        # Emergency stop (available immediately)
        emergency_stop,
    ])
