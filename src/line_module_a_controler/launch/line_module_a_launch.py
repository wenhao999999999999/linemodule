from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_module_a_controler',
            executable='line_module_a_home_service',
            name='line_module_a_home_service',
            output='screen'
        ),
        Node(
            package='line_module_a_controler',
            executable='line_module_a_controler',
            name='line_module_a_controler',
            output='screen'
        ),
    ])
