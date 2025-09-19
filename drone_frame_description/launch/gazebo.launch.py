import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from os.path import join

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('drone_frame_description')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'drone_frame.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    robot_World_file = os.path.join(pkg_ros_gz_rbot, 'worlds', 'omnicopter.sdf')

    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Start Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : f"-r -v 4 {robot_World_file}"
        }.items()
    )

    # Spawn Robot in Gazebo using ros_gz_sim with larger scale
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "drone_frame",
            "-allow_renaming", "true",
            "-z", "5.0",  # Much higher position to ensure visibility
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0",
            "-P", "0.0",
            "-R", "1.5708"
        ],            
        output='screen',
    )


    # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{
                    'config_file': ros_gz_bridge_config,
                }],
                output='screen'
            )
        ]
    )   

 

    # Add a longer delay before spawning to ensure everything is ready
    delayed_spawn = TimerAction(
        period=10.0,  # Wait 10 seconds for everything to be ready
        actions=[spawn]
    )

    return LaunchDescription(
        [
            # Nodes and Launches
            gazebo,
            start_gazebo_ros_bridge_cmd,
            robot_state_publisher,
            delayed_spawn,  # Use delayed spawn instead of immediate spawn
        ]
    )