#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import turtlesim.msg
import turtlesim.srv
import math
import random

class Turtle(object):
    def __init__(self, name):
        self.name = "turtle" + str(name)
        self.rate = rospy.Rate(10)
        self.pose = turtlesim.msg.Pose()
        self.pub_turtle_vel = rospy.Publisher(self.name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.sub_turtle_pose = rospy.Subscriber(self.name + '/pose', turtlesim.msg.Pose, self.handle_turtle_pose)

    def handle_turtle_pose(self, msg):
        self.pose = msg
    
    def spawn(self):
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        try:
            spawner(random.randint(1,9), random.randint(1,9), 0, self.name)
        except Exception as e:
            rospy.logerr(e)
            
    def move(self, linear, angular):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub_turtle_vel.publish(twist)
    
    def move_translate(self, x, y):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x
        twist.linear.y = y
        self.pub_turtle_vel.publish(twist)
    
    def stop(self):
        twist = geometry_msgs.msg.Twist()
        self.pub_turtle_vel.publish(twist)
