
# Tutorial 2: Introduction to ROS2 - some additional concepts

#### Development of Intelligent Systems, 2024

In this exercise you will get further familiarized with [ROS2](https://twitter.com/OpenRoboticsOrg/status/1629208251563929600). We will explore services,
writing nodes in Python, the usage of `ros2 launch` and `ros2 bag` commands as well as some
additional ROS 2 commands. 

Download the code for Tutorial 2 and extract the files in the `src` directory of your workspace. Build the package using `colcon build`. This will serve as a working example.

## Starting multiple nodes at once

In ROS we usually have multiple nodes running at the same time and it quickly becomes impractical to use `ros2 run` commands for starting each node separately. 

For these purposes we use the `ros2 launch` tool. This tool starts multiple nodes, as they are configured in a `.launch.py` script. Launch scripts can be set up in Python, XML, or YAML, but Python is the widely used standard. 

Run the following command in the terminal: `ros2 launch dis_tutorial2 example_launch.py` Which nodes were just started? (Hint: rqt_graph)

To familiarize yourself with some additional functionalities like setting parameters and remapping topic names see [the launch file documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

## Parameters

Sometimes we want to be able to store the values of certain parameters so that they are available to every node and can be reconfigured during operation. 

A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. ROS 2 makes this functionality available though `ros2 param set` and `ros2 param get` services.

Running `ros2 run rqt_reconfigure rqt_reconfigure` will also launch a GUI display of all parameters and allow them to be edited at runtime. Remember to write down any changes into the node's launch file since they will reset to their original values upon relaunching it.

## Recording and replaying topics

ROS contains the `ros2 bag` node which enables the recording and playback of messages posted to selected topics. This can be extremely useful for debugging purposes and enables us to develop programs without having access to the real robot.

- [ROS 2 bag documentation](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

## Achtung
You can find a simple variant of the popular game Achtung, die Kurve! implemented in turtlesim in script `achtung_die_turtles.py`. You can run it on your own, then analyze the code to see how the various parts of the gameplay loop are implemented in ROS2.


## RQT GUI tools 

`rqt` (starts the ROS2 inspection GUI, contains tools for visualizing topics and nodes, publishing messages, viewing images, plotting data, etc.). These tools can also be launched directly, e.g. `ros2 run rqt_reconfigure rqt_reconfigure`.

# Homework 2

## Writing a turtle mover

Your task is to create a service node which moves the simulated turtle from the turtlesim package. Check the [custom msgs and srvs documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) for help.

The service request should contain a string and an integer field. The string should be one of:

- "circle"
- "rectangle"
- "triangle"
- "random"

and the integer field specifies the duration in seconds. 

The node should then move the turtle in the specified trajectory for the given duration in the integer field. After the given duration the turtle should stop moving. The response to the client should contain a string field with the previous issued movement type.


