

# Tutorial 2: Introduction to ROS2 - some additional concepts

#### Development of Intelligent Systems, FRI, 2025/2026

In this exercise you will get further familiarized with [ROS2](https://twitter.com/OpenRoboticsOrg/status/1629208251563929600). We will explore services,
writing nodes in Python, the usage of `ros2 launch` and `ros2 bag` commands as well as some
additional ROS 2 commands. 

Download the code for Tutorial 2 and extract the files in the `src` directory of your workspace. Build the package using `colcon build`. This will serve as a working example.

## Starting multiple nodes at once

In ROS we usually have multiple nodes running at the same time and it quickly becomes impractical to use `ros2 run` commands for starting each node separately. 

For these purposes we use the `ros2 launch` tool. This tool starts multiple nodes, as they are configured in a `.launch.py` script. Launch scripts can be set up in Python, XML, or YAML, but Python is the widely used standard. 

Run the following command in the terminal: `ros2 launch dis_tutorial2 example_launch.py` Which nodes were just started? (Hint: rqt_graph)

If you want to reset the turtle position, you can use the service /reset using `ros2 service call /reset std_srvs/srv/Empty`

To familiarize yourself with some additional functionalities like setting parameters and remapping topic names see [the launch file documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

## Parameters

Sometimes we want to be able to store the values of certain parameters so that they are available to every node and can be reconfigured during operation. 

A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. ROS 2 makes this functionality available though `ros2 param set` and `ros2 param get` services.

Running `ros2 run rqt_reconfigure rqt_reconfigure` will launch a GUI display of all parameters and allow them to be edited at runtime. If you want to set custom initial parameter values, you should include them in the node's launch file since they will reset to their original values upon relaunching it otherwise.

If you want to set up new parameters in your custom node, you can use the function  `node.declare_parameter()` (see the source code `go_to_position_simple_node.py`). This should expose the parameters so other nodes, launch files and rqt are able to change them on the fly.

## Achieving a goal

In order to get your turtle or robot to a specific position in space, you must provide the appropriate movement commands. Launch the example using `ros2 launch dis_tutorial2 go_to_position_launch.py` and set a goal pose by publishing the appropriate message to topic `/goal_pose`. First, observe how the turtle moves to the goal position, then check the code to understand what happens in the background. Which are the phases of moving to the goal position and what happens in each of them?

## Recording and replaying topics

ROS contains the `ros2 bag` node which enables the recording and playback of messages posted to selected topics. This can be extremely useful for debugging purposes and enables us to develop programs without having access to the real robot.

- [ROS 2 bag documentation](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

You can start recording the messages using `ros2 bag record <topic name>`. The recordings will be saved in a directory named with the timestamp by default. You can also choose to record all the topics, but this could result in very large files, depending on the sensors used.

You can initiate the data replay using `ros2 bag play <directory name>`. Record some commands on topic `/cmd_vel` (with keyboard or via a node), then reset the turtle and replay the commands from the recording. If you need, the topic names can also be remapped at replay time.

## RQT GUI tools 

You can easily view the topics, services, nodes, etc. by using `rqt`. This command will open up the GUI where you can inspect topics, view the node graph and even call services or visualize topics.

# Homework 2

## Writing a turtle mover

Your task is to create a service node which moves the simulated turtle from the turtlesim package. Check the [custom msgs and srvs documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) for help.

The service request should contain a string and an integer field. The string should be one of:

- "circle"
- "rectangle"
- "triangle"
- "random"

and the integer field specifies the duration in seconds. 

The node should then move the turtle in the specified trajectory for the given duration in the integer field. After the given duration the turtle should stop moving. The response to the client should contain a string field with the previously issued movement type.


