#!/usr/bin/env python3

import rclpy
import numpy as np
import random

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange

class GoToPose(Node):

    def __init__(self, nodename='velocity_publisher', frequency=10):
        super().__init__(nodename)

        # General stuff, for controlling node execution
        self.timer_period = 1/frequency
        self.counter = 0

        # Variables for reaching the goal
        self.goal_phase = 0 # Possible phases are 0, 1 or 2
        self.goal_pose = None
        self.current_pose = None
        self.new_goal = False
        
        # ROS communications
        self.server = self.create_subscription(TurtlePose, '/goal_pose', self.set_goal, 10)
        self.server = self.create_subscription(TurtlePose, '/pose', self.new_pose, 1)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer to execute the node
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.lin_scale = 5.0
        self.ang_scale = 5.0

        desc = ParameterDescriptor(
            description="Linear speed multiplier for teleop",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)]
        )

        self.declare_parameter('linear_scale', self.lin_scale, desc)
        self.declare_parameter('angular_scale', self.ang_scale, desc)

        self.add_on_set_parameters_callback(self._on_params)

    def _on_params(self, params):
        for p in params:
            if p.name == 'linear_scale':
                # validate
                if p.type_ != p.Type.DOUBLE:
                    return SetParametersResult(successful=False, reason="linear speed must be a float")
                if p.value <= 0.0:
                    return SetParametersResult(successful=False, reason="linear speed must be > 0")
            elif p.name == 'angular_scale':
                # validate
                if p.type_ != p.Type.DOUBLE:
                    return SetParametersResult(successful=False, reason="angular speed must be a float")
                if p.value <= 0.0:
                    return SetParametersResult(successful=False, reason="angular speed must be > 0")

        # apply after validation
        for p in params:
            if p.name == 'linear_scale':
                self.lin_scale = float(p.value)
                self.get_logger().info(f"Updated speed -> {self.lin_scale}")
            elif p.name == 'angular_scale':
                self.ang_scale = float(p.value)
                self.get_logger().info(f"Updated speed -> {self.ang_scale}")

        return SetParametersResult(successful=True)

    ### Topic response functions
    def set_goal(self, posemsg):
        self.goal_pose = [posemsg.x, posemsg.y, posemsg.theta]
        self.new_goal = True
        self.get_logger().info(f"A goal has been set!")

    def new_pose(self, posemsg):
        if self.current_pose is None:
            self.get_logger().info(f"Just got a new current pose: {posemsg}")

        self.current_pose = [posemsg.x, posemsg.y, posemsg.theta]

    def publish_vel(self, lin, ang):
        cmd_msg = Twist()
        cmd_msg.linear.x = lin
        cmd_msg.angular.z = ang
        self.publisher.publish(cmd_msg)
        self.get_logger().info(f"I published a Twist command lin:{cmd_msg.linear.x}, ang:{cmd_msg.angular.z}")
        self.counter += 1

    ### Execution control functions
    def timer_callback(self):
        if self.new_goal and not self.current_pose is None:
            self.get_logger().info(f"Calculating a control action")
            lin, ang = self.get_command()
            print(lin, ang)
            self.publish_vel(lin, ang)
        self.counter += 1

    ### Utility functions
    def get_command(self):
        lin = 0.0
        ang = 0.0

        if self.goal_phase == 0: # First we send commands to turn towards the goal pose
            
            turtle_orientation = self.current_pose[2]
            goal_orientation = np.arctan2(self.goal_pose[1]-self.current_pose[1],
                                          self.goal_pose[0]-self.current_pose[0])
            angle_error = goal_orientation - turtle_orientation
            self.get_logger().info(f"Phase 1 of reaching a goal! Angular error is {angle_error}")
            if np.abs(angle_error)<0.05: # Target angle is reached!
                self.goal_phase = 1
                ang = 0.0
            else:
                if angle_error>0:
                    ang = self.ang_scale*0.1
                else:
                    ang = self.ang_scale*-0.1

        elif self.goal_phase == 1: # Then, we move towards the goal pose
            dist_to_goal = np.sqrt((self.current_pose[1] - self.goal_pose[1])**2 + 
                                   (self.current_pose[0] - self.goal_pose[0])**2)
            self.get_logger().info(f"Phase 2 of reaching a goal! Position error is {dist_to_goal}")
            if np.abs(dist_to_goal)<0.1: # Goal position is reached!
                self.goal_phase = 2
                lin = 0.0
            elif np.abs(dist_to_goal)<0.5:
                lin = self.lin_scale*0.05
            else:
                lin = self.lin_scale*0.1

        elif self.goal_phase == 2: # Finally, we should rotate the turtle in the goal rotation
            angle_error = self.goal_pose[2] - self.current_pose[2]
            self.get_logger().info(f"Phase 3 of reaching a goal! Angular error is {angle_error}")

            if np.abs(angle_error)<0.05: # Target angle is reached!
                self.get_logger().info(f"Goal has been reached!")
                self.goal_phase = 0
                self.new_goal = False
                ang = 0.0
            else:
                if angle_error>0:
                    ang = self.ang_scale*0.1
                else:
                    ang = self.ang_scale*-0.1

        return lin, ang


def main(args=None):
    rclpy.init(args=args)

    gtp = GoToPose("GoToPose")
    rclpy.spin(gtp)

    rclpy.shutdown()


if __name__=="__main__":
    main()