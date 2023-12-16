#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
from math import atan2, sqrt

# Key Config
BTN_NAME = ['X', 'A', 'B', 'Y']
BTN_NUM = [2, 0, 1, 3]

class turtle_controller(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.twist_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 100)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # JoyStick Initialization
        pygame.joystick.init()
        self.joystick0 = pygame.joystick.Joystick(0)
        self.joystick0.init()
        self.joystickx = 0
        self.joysticky = 0
        self.joystickz = 0
        pygame.init()

        # Initializing Default Count and Speed
        self.button_cnt = 0
        self.coff_linear = 1
        self.coff_angular = 1

        # Twist Initialization
        self.vel = Twist()
        self.vel.linear.x = float(0)
        self.vel.linear.y = float(0)
        self.vel.linear.z = float(0)
        self.vel.angular.x = float(0)
        self.vel.angular.y = float(0)
        self.vel.angular.z = float(0)
        self.twist_pub.publish(self.vel)

        # Print Operation Description
        print()
        print( f" LinearUP  {BTN_NAME[BTN_NUM[0]]}, LinearDown  {BTN_NAME[BTN_NUM[2]]}" )
        print( f" AngularUP {BTN_NAME[BTN_NUM[3]]}, AngularDonw {BTN_NAME[BTN_NUM[1]]}" )
        print()

    def timer_callback(self):
        eventlist = pygame.event.get()

        # Event Processing
        for e in eventlist:
            if e.type == QUIT:
                return

            # Stick Processing
            if e.type == pygame.locals.JOYAXISMOTION:
                # self.button_cnt += 1
                self.joystickx = -self.joystick0.get_axis(1)
                self.joysticky = -self.joystick0.get_axis(0)
                self.joystickz = -self.joystick0.get_axis(2)
                print(f"linear x: {self.joystickx * self.coff_linear:5.2f}, linear y: {self.joysticky * self.coff_linear:5.2f}, angular z: {self.joystickz * self.coff_angular:5.2f}")

            # Button Processing
            elif e.type == pygame.locals.JOYBUTTONDOWN:
                if   e.button == BTN_NUM[0]:
                    self.coff_linear += 0.1
                elif e.button == BTN_NUM[1]:
                    self.coff_linear -= 0.1
                elif e.button == BTN_NUM[2]:
                    self.coff_angular += 0.1
                elif e.button == BTN_NUM[3]:
                    self.coff_angular -= 0.1
                print(f"linear x: {self.coff_linear:5.2f}, angular z: {self.coff_angular:5.2f}")

        # Calculate Twist from stick input.
        self.vel.linear.x  = float( self.joystickx * self.coff_linear)
        self.vel.linear.y  = float( self.joysticky * self.coff_linear)
        self.vel.angular.z = float( self.joystickz * self.coff_angular)

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