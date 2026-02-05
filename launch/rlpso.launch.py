from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tiago_rlpso',
            executable='rlpso_node',
            output='screen'
        )
    ])

