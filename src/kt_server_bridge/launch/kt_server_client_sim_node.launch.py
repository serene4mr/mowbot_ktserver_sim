from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

ARGS = [
    DeclareLaunchArgument(
        'fb_min_lat',
        default_value='0.0',
        description='Minimum latitude for the feedback zone'
    ),
    DeclareLaunchArgument(
        'fb_max_lat',
        default_value='0.0',
        description='Maximum latitude for the feedback zone'
    ),          
    DeclareLaunchArgument(
        'fb_min_lon',
        default_value='0.0',
        description='Minimum longitude for the feedback zone'
    ),
    DeclareLaunchArgument(
        'fb_max_lon',
        default_value='0.0',
        description='Maximum longitude for the feedback zone'
    ),
]


def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('kt_server_bridge'),
        'config',
        'kt_server_bridge_sim.yaml'
    )

    return LaunchDescription(ARGS + [
        Node(
            package='kt_server_bridge',
            executable='kt_server_client_sim_node',
            name='kt_server_client_sim_node',
            output='screen',
            parameters=[
                config,
                {
                    'fb_min_lat': LaunchConfiguration('fb_min_lat'),
                    'fb_max_lat': LaunchConfiguration('fb_max_lat'),
                    'fb_min_lon': LaunchConfiguration('fb_min_lon'),
                    'fb_max_lon': LaunchConfiguration('fb_max_lon'),
                }
            ],
            remappings=[
                ('/gps/fix', '/gps/fix_filtered'),
                ('/odom', '/odom/local'),
            ],
        ),
        Node(
            package='kt_server_bridge',
            executable='kt_phase_trigger_node',
            name='kt_phase_trigger_node',
            output='screen',
        )
    ])