
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iris_camera',
            executable='iris_camera',
            name='iris_camera',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='iris_core',
            executable='iris_core',
            name='iris_core',
            output='screen',
            emulate_tty=True,
        ),
    ])