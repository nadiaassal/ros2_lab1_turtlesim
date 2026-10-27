from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtle_control',
            executable='turtle_manager',
            name='turtle_manager',
            parameters=[{
                'max_linear_speed': 2.0,
                'max_angular_speed': 2.0
            }]
        )
    ])
