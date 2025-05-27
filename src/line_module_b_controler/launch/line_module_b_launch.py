from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_module_b_controler',
            executable='line_module_b_home_service',  # 对应 home_service 脚本
            name='line_module_b_home_service',
            output='screen'
        ),
        Node(
            package='line_module_b_controler',
            executable='line_module_b_controler',  # ✅ 改成和 setup.py 中保持一致
            name='line_module_b_controler',
            output='screen'
        )
    ])
