# These are the commands that were used during the tutorial. This file should not be run, it is to be used as a reference.
# Also there might be wrong commands.
cd ROS2
colcon build --help
colcon build --symlink-install
cd ROS2
cd src
cd dis_tutorial2
cd launch/
ls
ros2 launch example_launch.py 
cd ~
ros2 run turtlesim turtlesim_node 
ros2 launch example_launch.py 
ros2 launch dis_tutorial2 example_launch.py
ros2 launch dis_tutorial2 go_to_position_launch.py 
cd ROS2/src/dis_tutorial2/launch/
ls
ros2 launch example_launch.yaml 
cd ~
ros2 launch dis_tutorial2 example_launch.py
ros2 launch dis_tutorial2 go_to_position_launch.py 
ros2 node list
ros2 topic list
ros2 topic pub --once /goal_pose turtlesim/msg/Pose "{x: 7., y: 7., theta: 0., linear_velocity: 0., angular_velocity: 0.}"
ros2 node list
ros2 topic list
cd ~
ros2 launch dis_tutorial2 example_launch.py
ros2 topic list
ros2 topic info /turtle1/cmd_vel 
ros2 interface show --help
ros2 interface show geometry_msgs/msg/Twist
ros2 param list
ros2 param set --help
ros2 param set /random_velocity_publisher_node scale_angular 5.
ros2 topic pub --once /goal_pose turtlesim/msg/Pose "{x: 7., y: 7., theta: 0., linear_velocity: 0., angular_velocity: 0.}"
ros2 node list
ros2 topic list
ros2 topic echo /turtle1/cmd_vel 
ros2 topic info /turtle1/cmd_vel 
ros2 interface show geometry_msgs/msg/Twist
ros2 param list
ros2 param set /random_velocity_publisher_node scale_angular 5.