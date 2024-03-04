from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='dis_tutorial2',
            executable='go_to_position',
            name='go_to_position_node',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel'),
                ('/pose', '/turtle1/pose'),
            ]
        )
    ])