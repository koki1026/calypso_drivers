# propeller_driver/launch/propeller_driver_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) joy_node を先に立ち上げ
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                # 必要に応じてデバイスやデッドゾーンを変えられるように
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
            }]
        ),

        # 2) PropellerController ノード
        Node(
            package='propeller_driver',
            executable='propeller_driver',
            name='propeller_controller',
            output='screen',
            remappings=[
                # joy のトピック名を変えているならここで合わせる
                # ('joy', '/joy'),
            ]
        ),
    ])