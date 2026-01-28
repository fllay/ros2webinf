from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    Generate a ROS2 launch description to start the Nav2 stack.
    It includes the standard bringup launch and sets custom parameters and map files.
    """
    # Locate the Nav2 bringup launch file from the standard package
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    # Path to the custom navigation parameters (tuning for the specific robot)
    nav2_config_path = '/home/pi/amr_configs/nav2_parameter.yaml'
    
    # Declare the 'map' argument, allowing users to specify different maps at runtime
    map_path_arg = DeclareLaunchArgument(
        name='map',
        default_value='/home/pi/amr_configs/maps/house2ndFloorNew.yaml',
        description='Full path to the map yaml file'
    )
    
    # Declare the 'sim' argument for simulation vs real-world time
    sim_arg = DeclareLaunchArgument(
        name='sim',
        default_value='false',
        description='Enable use_sim_time'
    )

    # Include the standard Nav2 bringup launch with our custom arguments
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'map': LaunchConfiguration("map"),
            'use_sim_time': LaunchConfiguration("sim"),
            'params_file': nav2_config_path
        }.items()
    )

    return LaunchDescription([
        map_path_arg,
        sim_arg,
        nav2_launch
    ])
