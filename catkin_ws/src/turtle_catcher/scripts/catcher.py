#!/usr/bin/env python

import rospy
import random
import tf
import geometry_msgs.msg
import turtlesim.msg
import turtlesim.srv
import math
import sys

class Catcher:
    def __init__(self, number):
        self.name = 'catcher' + str(number)
        # rospy.init_node(self.name)
        self.rate = rospy.Rate(10)
        self.catcher_pose_broadcast = tf.TransformBroadcaster()
        self.target_pose_listerner = tf.TransformListener()
        # self.target_pose_listerner.waitForTransform('/turtle1', '/target', rospy.Time(), rospy.Duration(4.0))
        self.sub_target_pose = rospy.Subscriber('/turtle1/pose', turtlesim.msg.Pose, self.handle_target_pose)
        self.sub_current_pose = rospy.Subscriber(self.name + '/pose', turtlesim.msg.Pose, self.handle_current_pose)
        self.pub_catcher_vel = rospy.Publisher(self.name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.pub_global_best_pose = rospy.Publisher('global_best_pose', turtlesim.msg.Pose, queue_size=1)
        # self.sub_global_best_pose = rospy.Subscriber('global_best_pose', turtlesim.msg.Pose, self.handle_global_best_pose)

        self.rate = rospy.Rate(10)

        self.threshold = 0.1 # Distance threshold to stop the turtle
        self.max_moving_distance = 1 # Maximum distance the turtle can move in one step

        # Proportional gain for the controller
        self.K_linear = 1
        self.K_angular = 4.0

        # PSO parameters
        self.c1 = 2.0
        self.c2 = 2.0
        
        # Personal best
        self.pbest_pose = turtlesim.msg.Pose()
        self.pbest_value = sys.maxsize

        # Global best
        self.gbest_pose = turtlesim.msg.Pose()
        self.gbest_value = sys.maxsize

        # Poses
        self.target_pose = turtlesim.msg.Pose()
        self.current_pose = turtlesim.msg.Pose()

    def spawn(self):
        # rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        try:
            spawner(random.randint(1,9), random.randint(1,9), 0, self.name)
        except Exception as e:
            rospy.logerr(e)
    
    # Calculate fitness function of the given pose
    def fitness_function(self, pose):
        fitness = distance = math.sqrt((pose.x - self.target_pose.x)**2 + (pose.y - self.target_pose.y)**2)
        # distance = max(distance, 0.0001)
        # fitness = 1 / distance
        return fitness

    # self target_pose is handle every time the target pose is updated
    def handle_target_pose(self, msg):
        self.target_pose = msg
        rospy.loginfo_once("Target pose: %f, %f", self.target_pose.x, self.target_pose.y)

    # self pbest is handle every time the current pose is updated
    def handle_current_pose(self, msg):
        self.current_pose = msg
        rospy.loginfo_once("Current pose: %f, %f", self.current_pose.x, self.current_pose.y)
        fitness = self.fitness_function(self.current_pose)
        if fitness < self.pbest_value:
            self.pbest_value = fitness
            self.pbest_pose = self.current_pose
            rospy.loginfo_once("Pbest pose: %f, %f", self.pbest_pose.x, self.pbest_pose.y)
        # Publish global best pose
        if fitness < self.gbest_value:
            self.pub_global_best_pose.publish(self.current_pose)
    
    # self gbest is handle every time the global best pose is updated
    def handle_global_best_pose(self, msg):
        # Update the global best pose and its fitness value
        self.gbest_pose = msg
        self.gbest_value = self.fitness_function(msg)
        print(self.gbest_value)
        
    # Calculate distance between target and self position
    def calculate_distance(self, target):
        return math.sqrt((self.current_pose.x - target.x)**2 + (self.current_pose.y - target.y)**2)
        
    # Calculate angle between target and self position
    def calculate_angle(self, target):
        return math.atan2(target.y - self.current_pose.y, target.x - self.current_pose.x) 
    
    # Calculate velocity and go to target position
    def calculate_velocity(self, target):
        distance = self.calculate_distance(target)
        angle = self.calculate_angle(target)
        vel = geometry_msgs.msg.Twist()
        if distance > self.threshold:
            vel.linear.x = self.K_linear * distance * math.cos(angle)
            vel.linear.y = self.K_linear * distance * math.sin(angle)
            # vel.angular.z = self.K_angular * (angle - self.current_pose.theta)
        else:
            vel.linear.x = 0
            vel.linear.y = 0
            # vel.angular.z = 0
        self.pub_catcher_vel.publish(vel)
    
    # Go to target position
    def go_to_target(self, target):
        dx_target = target.x - self.current_pose.x
        dy_target = target.y - self.current_pose.y

        dx_pbest = self.pbest_pose.x - self.current_pose.x
        dy_pbest = self.pbest_pose.y - self.current_pose.y
        rand_c1 = random.random()
        rand_c2 = random.random()
        dx_new = self.c1 * rand_c1 * dx_pbest + self.c2 * rand_c2 * dx_target
        dy_new = self.c1 * rand_c1 * dy_pbest + self.c2 * rand_c2 * dy_target
        
        # distance_target = math.sqrt(dx_target**2 + dy_target**2)
        # distance_pbest = math.sqrt(dx_pbest**2 + dy_pbest**2)
        distance_new = math.sqrt(dx_new**2 + dy_new**2)

        target_adj = turtlesim.msg.Pose()
        target_adj.x = self.current_pose.x + dx_new
        target_adj.y = self.current_pose.y + dy_new

        # If the distance is smaller than the maximum moving distance, go to the adjusted target directly
        if distance_new < self.max_moving_distance:
            while self.calculate_distance(target_adj) > self.threshold:
                self.calculate_velocity(target_adj)
            return
        
        theta = math.atan2(dy_new, dx_new)

        # Calculate the corrected target position
        regularized_target = turtlesim.msg.Pose()
        regularized_target.x = self.current_pose.x + self.max_moving_distance * math.cos(theta)
        regularized_target.y = self.current_pose.y + self.max_moving_distance * math.sin(theta)
        while self.calculate_distance(regularized_target) > self.threshold:
            self.calculate_velocity(regularized_target)
