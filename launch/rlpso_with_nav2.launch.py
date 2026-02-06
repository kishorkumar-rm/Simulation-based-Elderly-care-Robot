from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    rlpso = Node(
        package='tiago_rlpso',
        executable='rlpso_waypoint',
        output='screen'
    )

    return LaunchDescription([
        nav2,
        rlpso
    ])

