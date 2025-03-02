from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_nav_controller',  # 这里的包名必须和创建的ROS2包名一致
            executable='send_goal',  # 对应 setup.py 里 entry_points 里的名字
            name='navigation_client',
            output='screen'
        )
    ])
