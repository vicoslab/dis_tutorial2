#!/usr/bin/env python3

import rclpy
import numpy as np
import random

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from turtlesim.msg import Color
from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Empty

import sys
import tty
import termios
import threading

class GameNode(Node):

    def __init__(self, nodename='game_node', frequency=10, def_lin_vel=0.8, ang_inc=0.3):
        super().__init__(nodename)

        # General stuff, for controlling node execution
        self.timer_period = 1/frequency

        # Some needed variables
        self.t1_twist = Twist()
        self.t2_twist = Twist()
        self.def_lin_vel = def_lin_vel
        self.ang_inc = ang_inc
        self.t1_twist.linear.x = self.t2_twist.linear.x = self.def_lin_vel
        self.t1_ang_vel = self.t2_ang_vel = 0.
        self.game_started = False
        self.new_key = False
        self.last_key = ''

        self.turtle1_positions = []
        self.turtle2_positions = []

        

        # ROS service clients
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.clear_client = self.create_client(Empty, '/clear')

        # Timer to execute the node
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Start a separate thread to listen for keypresses
        self.thread = threading.Thread(target=self.read_keypresses)
        self.thread.daemon = True
        self.thread.start()

        request = Empty.Request()        
        future = self.clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(f"clear turtle")
        else:
            print("Failed to clear turtle")

        self.kill_turtle('turtle1')
        self.kill_turtle('turtle2')
        self.spawn_turtle(4., 1., np.pi/2, 'turtle1')
        self.spawn_turtle(7., 1., np.pi/2, 'turtle2')
        self.set_pen_color('turtle1', 255,0,0,width=5)
        self.set_pen_color('turtle2', 0,0,255,width=5)
        self.get_logger().info(f'Turtles are initialized! Use A, S keys to turn the left turtle and J, K keys to turn the second turtle')
        self.game_started = True
        self.game_over = False

        # ROS Subscribers and publishers
        self.t1pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.t2pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_subscription(TurtlePose, 'turtle1/pose', self.turtle1_callback, 10)
        self.create_subscription(TurtlePose, 'turtle2/pose', self.turtle2_callback, 10)

        self.create_timer(0.1, self.check_intersections)

    def turtle1_callback(self, msg: TurtlePose):
        self.turtle1_positions.append((msg.x, msg.y))

    def turtle2_callback(self, msg: TurtlePose):
        self.turtle2_positions.append((msg.x, msg.y))

    def set_pen_color(self, turtle_name, r, g, b, width=3, off=0):
        """
        Sets the pen color for the given turtle using the turtlesim SetPen service.
        :param turtle_name: Name of the turtle (e.g., 'turtle1')
        :param r: Red component (0-255)
        :param g: Green component (0-255)
        :param b: Blue component (0-255)
        :param width: Width of the pen
        :param off: 0 to enable the pen, 1 to disable
        """
        service_name = f"/{turtle_name}/set_pen"
        client = self.create_client(SetPen, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {service_name} service...")
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Set {turtle_name} pen to color ({r}, {g}, {b})")
        else:
            self.get_logger().error(f"Failed to call {service_name} service")

    def read_keypresses(self):
        """ Reads keyboard input without blocking """
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())  # Set terminal to raw mode
            self.running = True
            while self.running and rclpy.ok():
                key = sys.stdin.read(1)  # Read one character
                # print(f'{key}')
                self.last_key = key
                self.new_key = True
                if key == 'q':  # Example: Quit on 'q' press
                    self.get_logger().info('Exiting...')
                    self.running = False
                    return
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)  # Restore terminal settings


    def spawn_turtle(self, x, y, theta, name):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(f"Spawned turtle: {future.result().name}")
        else:
            print("Failed to spawn turtle")

    def kill_turtle(self, name):
        request = Kill.Request()
        request.name = name
        
        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(f"Killed turtle")
        else:
            print("Failed to kill turtle")

    def send_movement_command(self, turtle_name, x, z):
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.angular.z = z

        if self.running:
            if turtle_name=='turtle1':
                self.t1pub.publish(twist_msg)
            elif turtle_name=='turtle2':
                self.t2pub.publish(twist_msg)

    def publish_vel(self, lin, ang):
        if self.running:
            cmd_msg = Twist()
            cmd_msg.linear.x = lin
            cmd_msg.angular.z = ang
            self.publisher.publish(cmd_msg)
            # self.get_logger().info(f"I published a Twist command lin:{cmd_msg.linear.x}, ang:{cmd_msg.angular.z}")
            self.counter += 1

    def update_movement_commands(self, key):
        self.new_key = False

        if key=='a':
            self.t1_ang_vel += self.ang_inc
        elif key=='s':
            self.t1_ang_vel -= self.ang_inc
        elif key=='k':
            self.t2_ang_vel += self.ang_inc
        elif key=='l':
            self.t2_ang_vel -= self.ang_inc

    ### Execution control functions
    def timer_callback(self):
        if self.game_started:
            if self.new_key:
                self.update_movement_commands(self.last_key)

            # self.get_logger().info(f"Sending new actions!")
            self.send_movement_command('turtle1', self.def_lin_vel, self.t1_ang_vel)
            self.send_movement_command('turtle2', self.def_lin_vel, self.t2_ang_vel)
        # else:
        #     self.get_logger().info(f"Waiting for the game to start!")

        if not self.running:
            self.thread.join()
            self.destroy_node()
            exit()

    def on_segment(self, p, q, r):
        """Check if point q lies on the line segment pr."""
        if (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1])):
            return True
        return False

    def orientation(self, p, q, r):
        """Return the orientation of the triplet (p, q, r).
        0 --> Collinear
        1 --> Clockwise
        2 --> Counterclockwise
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(val) < 1e-6:
            return 0  # collinear
        return 1 if val > 0 else 2

    def segments_intersect(self, p1, p2, q1, q2):
        """Return True if line segment p1-p2 intersects with q1-q2."""
        o1 = self.orientation(p1, p2, q1)
        o2 = self.orientation(p1, p2, q2)
        o3 = self.orientation(q1, q2, p1)
        o4 = self.orientation(q1, q2, p2)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Special cases
        if o1 == 0 and self.on_segment(p1, q1, p2):
            return True
        if o2 == 0 and self.on_segment(p1, q2, p2):
            return True
        if o3 == 0 and self.on_segment(q1, p1, q2):
            return True
        if o4 == 0 and self.on_segment(q1, p2, q2):
            return True

        return False

    def check_intersections(self):
        """Check each segment of turtle1's trail against each segment of turtle2's trail."""
        # Loop over each consecutive pair (segment) in turtle1's trail.
        for i in range(1, len(self.turtle1_positions)):
            seg1_start = self.turtle1_positions[i - 1]
            seg1_end = self.turtle1_positions[i]
            # And check against every segment in turtle2's trail.
            for j in range(1, len(self.turtle2_positions)):
                seg2_start = self.turtle2_positions[j - 1]
                seg2_end = self.turtle2_positions[j]
                if self.segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end):
                    self.get_logger().info('Trails intersect!')
                    print(f'{seg1_start=}')
                    print(f'{seg1_end=}')
                    print(f'{seg2_start=}')
                    print(f'{seg2_end=}')
                    # Once an intersection is found, you can take action and break early.
                    self.running = False
                    self.thread.join()
                    self.destroy_node()
                    exit()
                    return


def main(args=None):
    rclpy.init(args=args)

    gtp = GameNode("turtle_game_node", 100)
    
    rclpy.spin(gtp)

    gtp.running = False
    gtp.thread.join()
    gtp.destroy_node()
    rclpy.shutdown()
    rclpy.shutdown()


if __name__=="__main__":
    main()