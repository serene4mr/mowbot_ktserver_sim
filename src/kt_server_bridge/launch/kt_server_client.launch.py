from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('kt_server_bridge'),
        'config',
        'kt_server_bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='kt_server_bridge',
            executable='kt_server_client_node',
            name='kt_server_client_node',
            output='screen',
            parameters=[config],
            remappings=[
                ('/gps/fix', '/gps/fix_filtered'),
                ('/odometry/filtered', '/odometry/filtered'),
                ('/heading', '/gps/heading'),
            ],
        ),
    ])