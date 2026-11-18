import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('underwater_motor_sim'),
        'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='underwater_motor_sim',
            executable='motor_sim_node',
            name='motor_sim_node',
            parameters=[params],
            output='screen',
        ),
    ])
