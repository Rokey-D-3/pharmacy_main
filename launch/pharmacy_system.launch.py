from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pharmacy_main',
            executable='pharmacy_manager',
            name='pharmacy_manager',
            output='screen'
        ),
        Node(
            package='pharmacy_main',
            executable='pharmacy_gui',
            name='pharmacy_gui',
            output='screen'
        ),
        Node(
            package='pharmacy_main',
            executable='voice_input',
            name='voice_input',
            output='screen'
        ),
        Node(
            package='pharmacy_main',
            executable='symptom_matcher',
            name='symptom_matcher',
            output='screen'
        ),
        Node(
            package='pharmacy_main',
            executable='detector',
            name='detector',
            output='screen'
        ),
        Node(
            package='pharmacy_main',
            executable='robot_arm',
            name='robot_arm',
            output='screen'
        ),
    ])
