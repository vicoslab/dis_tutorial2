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
            executable='random_velocity_publisher',
            name='random_velocity_publisher_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'scale_linear':0.98,
                 'scale_angular':1.02}
            ],
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel'),
            ]
        )
    ])