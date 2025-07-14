from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    この関数がLaunch時にROS2によって呼び出される。
    起動したいノードのリストを返す。
    """
    return LaunchDescription([
        
        Node(
            package='image_converter_pkg',
            executable='mic_repub_node',
            name='mic_republisher'
        ),

        Node(
            package='image_converter_pkg',
            executable='image_repub_node',
            name='image_republisher'
        ),

    ])