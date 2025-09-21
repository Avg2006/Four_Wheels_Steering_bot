from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            respawn=True,
            parameters=[{'dev': '/dev/input/js0'}],
            remappings=[('joy', 'joy')]
        ),
        Node(
            package='joy',
            executable='joy_node', 
            name='joy_arm',
            respawn=True,
            parameters=[{'dev': '/dev/input/js1'}],
            remappings=[('joy', 'joy_arm')]
        )
    ])
