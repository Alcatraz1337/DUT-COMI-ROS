#!/usr/bin/env python

import rospy
import turtlesim.msg
import turtlesim.srv
import geometry_msgs.msg
import random
import math
from turtle_base import Turtle

class PSO_Catcher(Turtle):
    def __init__(self, name):
        super(PSO_Catcher, self).__init__(name)
        self.threshold = 0.1
        self.max_moving_distance = 1.0
        # A subscriber for the next moving target pose
        self.sub_target_pose = rospy.Subscriber(self.name + '/target_pose', turtlesim.msg.Pose, self.callback_target_pose)
        self.sub_turtle_pose = rospy.Subscriber(self.name + '/pose', turtlesim.msg.Pose, self.handle_turtle_pose)

        # Proportional gain for the controller
        self.K_linear = 1
        self.K_angular = 4.0

        # A flag for marking if the turtle has reached the target
        self.is_reached = False
        self.last_vel = geometry_msgs.msg.Twist()

    # Calculate distance between given target pose and current pose
    def calculate_distance(self, target_pose):
        return math.sqrt((target_pose.x - self.pose.x)**2 + (target_pose.y - self.pose.y)**2)
    
    # Calculate angle between given target pose and current pose
    def calculate_angle(self, target_pose):
        return math.atan2(target_pose.y - self.pose.y, target_pose.x - self.pose.x)
    
    # Calculate velocity of the turtle
    def calculate_velocity(self, target_pose):
        distance = self.calculate_distance(target_pose)
        angle = self.calculate_angle(target_pose)
        vel = geometry_msgs.msg.Twist()

        vel.linear.x = self.K_linear * distance * math.cos(angle)
        vel.linear.y = self.K_linear * distance * math.sin(angle)

        self.last_vel = vel
        
        return vel

    def execute_vel_cmd(self, vel):
        self.is_reached = False
        self.pub_turtle_vel.publish(vel)
        self.rate.sleep()
    
    def move_to_target(self, target_pose):
        # print("Moving to target pose: x = {}, y = {}".format(target_pose.x, target_pose.y))
        while True:
            vel = self.calculate_velocity(target_pose)
            self.pub_turtle_vel.publish(vel)
            self.rate.sleep()
            if self.calculate_distance(target_pose) < self.threshold:
                self.stop()
                break
        # print("Reached target pose: x = {}, y = {}".format(target_pose.x, target_pose.y))
        self.is_reached = True

    def callback_target_pose(self, msg):
        print("Received target pose: x = {}, y = {}".format(msg.x, msg.y))
        self.move_to_target(msg)

    def handle_turtle_pose(self, msg):
        if msg.linear_velocity == 0.0:
            self.is_reached = True
        return super(PSO_Catcher, self).handle_turtle_pose(msg)