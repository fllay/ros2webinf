from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    nav2_config_path = '/home/pi/amr_configs/navigation_map.yaml'
    
    # We will pass the map path via argument
    map_path_arg = DeclareLaunchArgument(
        name='map',
        default_value='/home/pi/amr_configs/maps/off1.yaml',
        description='Full path to the map yaml file'
    )
    
    sim_arg = DeclareLaunchArgument(
        name='sim',
        default_value='false',
        description='Enable use_sim_time'
    )

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
