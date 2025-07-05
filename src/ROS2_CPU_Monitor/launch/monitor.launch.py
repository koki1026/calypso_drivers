
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    CPU/メモリ監視ノードを起動するLaunchDescriptionを生成します。
    """
    return LaunchDescription([
        Node(
            package='cpu_monitor',
            executable='publisher.py',
            name='cpu_memory_monitor_node',
            output='screen'
        )
    ])