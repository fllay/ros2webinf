from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    Generate a ROS2 launch description to start the Nav2 stack with optional Keepout Filters.
    """
    # Locate the Nav2 bringup launch file
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    # Declare launch arguments
    map_path_arg = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(os.getcwd(), 'amr_configs/maps/house2ndfloor.yaml'),
        description='Full path to the map yaml file'
    )

    params_file_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(os.getcwd(), 'amr_configs/navigation_map.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    
    sim_arg = DeclareLaunchArgument(
        name='sim',
        default_value='false',
        description='Enable use_sim_time'
    )
    
    use_keepout_arg = DeclareLaunchArgument(
        name='use_keepout',
        default_value='false',
        description='Whether to use keepout filters'
    )

    # Path to keepout mask (derived from map name)
    # Note: This simple logic assumes the mask exists at [map_name]_mask.yaml
    # In a more robust setup, you might pass this as an explicit argument.
    mask_yaml_file = [LaunchConfiguration('map'), '_mask.yaml']

    # Keepout Filter Servers
    keepout_servers = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_keepout')),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                parameters=[{'yaml_filename': mask_yaml_file}],
                remappings=[
                    ('map', 'filter_mask'),
                    ('map_server_on_lifecycle', 'filter_mask_server_on_lifecycle')
                ]
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                parameters=[{
                    'type': 0, # Keepout filter
                    'filter_info_topic': '/costmap_filter_info',
                    'mask_topic': '/filter_mask',
                    'base_frame_id': 'map',
                    'flip_x': False,
                    'flip_y': False
                }]
            ),
            # Manager to handle the lifecycle of these nodes
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_filter',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('sim'),
                    'autostart': True,
                    'node_names': ['filter_mask_server', 'costmap_filter_info_server']
                }]
            )
        ]
    )

    # Standard Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'map': LaunchConfiguration("map"),
            'use_sim_time': LaunchConfiguration("sim"),
            'params_file': LaunchConfiguration("params_file")
        }.items()
    )

    return LaunchDescription([
        map_path_arg,
        params_file_arg,
        sim_arg,
        use_keepout_arg,
        keepout_servers,
        nav2_launch
    ])
