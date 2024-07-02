from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pkg',
            namespace='camera',
            executable='camera_node',
            name='test'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener'
        )

    ])