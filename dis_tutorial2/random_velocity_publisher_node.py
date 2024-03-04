#!/usr/bin/env python3

import rclpy
import random

from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):

    def __init__(self, nodename='velocity_publisher', frequency=5):
        super().__init__(nodename)

        self.timer_period = 1/frequency
        self.counter = 0

        self.declare_parameter('scale_linear', 1.)
        self.declare_parameter('scale_angular', 1.)
        
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def publish_vel(self):
        # Read the parameter values
        scale_linear_param = self.get_parameter('scale_linear')
        scale_angular_param = self.get_parameter('scale_angular')

        scale_linear = scale_linear_param.value
        scale_angular = scale_angular_param.value

        cmd_msg = Twist()
        cmd_msg.linear.x = scale_linear * random.random()
        cmd_msg.angular.z = scale_angular * (random.random() - 0.5)
        self.publisher.publish(cmd_msg)
        self.get_logger().info(f"The parameters are: scale_linear {scale_linear}, scale_angular:{scale_angular}")
        self.get_logger().info(f"I published a Twist command lin:{cmd_msg.linear.x}, ang:{cmd_msg.angular.z}")

    def timer_callback(self):
        self.publish_vel()
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)

    vp = VelocityPublisher("velocity_publisher")
    rclpy.spin(vp)

    rclpy.shutdown()


if __name__=="__main__":
    main()