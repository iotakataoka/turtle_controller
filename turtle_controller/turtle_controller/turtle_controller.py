import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
from math import atan2, sqrt

class turtle_controller(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.twist_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 100)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # joyStick Initialization
        pygame.joystick.init()
        self.joyStick = pygame.joystick.Joystick(0)
        self.joyStick.init()
        self.joyStick_x = 0
        self.joyStick_y = 0
        self.joyStick_z = 0
        pygame.init()

        # Initializing Default Count and Speed
        self.linerVel = 1
        self.angularVel = 1

        # Twist Initialization
        self.vel = Twist()
        self.vel.linear.x = float(0)
        self.vel.linear.y = float(0)
        self.vel.linear.z = float(0)
        self.vel.angular.x = float(0)
        self.vel.angular.y = float(0)
        self.vel.angular.z = float(0)
        self.twist_pub.publish(self.vel)

    def timer_callback(self):
        eventlist = pygame.event.get()

        # Event Processing
        for e in eventlist:
            if e.type == QUIT:
                return

            # Stick Processing
            if e.type == pygame.locals.JOYAXISMOTION:
                self.joyStick_x = -self.joyStick.get_axis(1)
                self.joyStick_y = -self.joyStick.get_axis(0)
                self.joyStick_z = -self.joyStick.get_axis(2)
                print(f"linear x: {self.joyStick_x * self.linerVel: 5.2f}, linear y: {self.joyStick_y * self.linerVel: 5.2f}, angular z: {self.joyStick_z * self.angularVel: 5.2f}")

            # Button Processing
            elif e.type == pygame.locals.JOYBUTTONDOWN:
                if   e.button == 3: # X
                    self.linerVel += 0.1
                elif e.button == 0: # A
                    self.linerVel -= 0.1
                elif e.button == 1: # A
                    self.angularVel += 0.1
                elif e.button == 2: # Y
                    self.angularVel -= 0.1
                print(f"linearVel: {self.linerVel: .2f}, angularVel: {self.angularVel: .2f}")

        # Calculate Twist from stick input.
        self.vel.linear.x  = float( self.joyStick_x * self.linerVel)
        self.vel.linear.y  = float( self.joyStick_y * self.linerVel)
        self.vel.angular.z = float( self.joyStick_z * self.angularVel)

        # Publish Twist
        self.twist_pub.publish(self.vel)

def ros_main(args=None):
    rclpy.init(args=args)

    turtle_controller_class = turtle_controller()
    rclpy.spin(turtle_controller_class)

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros_main()