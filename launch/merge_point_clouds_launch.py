from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='merge_point_clouds',
            executable='merge_point_clouds',
            output="screen",

            # arguments=['--ros-args', '--log-level', 'DEBUG']
        
        )
    ])