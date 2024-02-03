import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()
    log_level = "warn"

    config = os.path.join(
        get_package_share_directory('px4_offboard'),
        'config',
        'params.yaml'
        )
    
    node = Node(
        package='px4_offboard',
        executable='px4_offboard',
        name='px4_offboard',
        parameters=[config],
        output='screen',
        #arguments=['--ros-args', '--log-level', log_level]
    )

    launch_description.add_action(node)
    return launch_description
