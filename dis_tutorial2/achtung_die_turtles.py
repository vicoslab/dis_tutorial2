#!/usr/bin/env python3

import rclpy
import numpy as np
import random

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from turtlesim.msg import Color
from turtlesim.srv import Spawn, Kill

import sys
import tty
import termios
import threading

class GameNode(Node):

    def __init__(self, nodename='game_node', frequency=1, def_lin_vel=0.8, ang_inc=0.3):
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

        # ROS Subscribers and publishers
        self.t1sub = self.create_subscription(Color, '/turtle1/color_sensor', self.t1_read_color, 10)
        self.t2sub = self.create_subscription(Color, '/turtle2/color_sensor', self.t2_read_color, 10)
        self.t1pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.t2pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # ROS service clients
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')

        # Timer to execute the node
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Start a separate thread to listen for keypresses
        self.thread = threading.Thread(target=self.read_keypresses)
        self.thread.daemon = True
        self.thread.start()

        # Finally restart the sim
        self.kill_turtle('turtle1')
        self.spawn_turtle(4., 1., np.pi/2, 'turtle1')
        self.spawn_turtle(7., 1., np.pi/2, 'turtle2')
        self.get_logger().info(f'Turtles are initialized! Use A, S keys to turn the left turtle and J, K keys to turn the second turtle')
        self.game_started = True

    def t1_read_color(self, msg):
        if self.game_started:
            if not (msg.r==179 and msg.g==184 and msg.b==255):
                print(msg.r,msg.g,msg.b)
        #     print('GAME OVER! Turtle 2 has WON the game!')
        #     self.running = False
        #     self.thread.join()
        #     self.destroy_node()
        #     rclpy.shutdown()

    def t2_read_color(self, msg):
        if self.game_started:
            if not (msg.r==179 and msg.g==184 and msg.b==255):
                print(msg.r,msg.g,msg.b)
        # if not (msg.r==179 and msg.g==184 and msg.b==255):
        #     print('GAME OVER! Turtle 1 has WON the game!')
        #     self.running = False
        #     self.thread.join()
        #     self.destroy_node()
        #     rclpy.shutdown()

    def read_keypresses(self):
        """ Reads keyboard input without blocking """
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())  # Set terminal to raw mode
            self.running = True
            while self.running and rclpy.ok():
                key = sys.stdin.read(1)  # Read one character
                print(f'{key}')
                self.last_key = key
                self.new_key = True
                if key == 'q':  # Example: Quit on 'q' press
                    self.get_logger().info('Exiting...')
                    self.running = False
                    rclpy.shutdown()
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

        if turtle_name=='turtle1':
            self.t1pub.publish(twist_msg)
        elif turtle_name=='turtle2':
            self.t2pub.publish(twist_msg)

    def publish_vel(self, lin, ang):
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