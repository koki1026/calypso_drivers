from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1つ目のノード (同じパッケージ内)
        Node(
            package='image_converter_pkg',
            executable='image_repub_node',
            name='image_repub_node'
        ),
        
        # 2つ目のノード (同じパッケージ内)
        Node(
            package='image_converter_pkg',
            executable='mic_repub_node', # 別の実行可能ファイル名
            name='mic_repub_node'
        ),
        
    ])
