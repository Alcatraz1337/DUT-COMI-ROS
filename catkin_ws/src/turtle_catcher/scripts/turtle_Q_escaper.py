#!/usr/bin/env python

import rospy
import turtlesim.msg
import turtlesim.srv
import geometry_msgs.msg
import random
import math
from turtle_base import Turtle
import numpy as np

class Q_Escaper(Turtle):
    def __init__(self, name):
        super(Q_Escaper, self).__init__(name)

        # Q-Learning Parameters
        self.current_state = tuple() # Current state [angle, distance]
        self.alpha = 0.1 # Learning rate
        self.epsilon = 0.1 # Epsilon-greedy algorithm parameter for choosing action. The probability of choosing a random action
        self.gamma = 0.9 # Discount factor
        self.action_size = 8
        self.angle_step_size = 45
        self.distance_step_size = 1.0
        self.max_moving_distance = 1.0
        self.max_distance_threshold = 5.0

        self.is_reached = True
        self.sub_turtle_pose = rospy.Subscriber(self.name + '/pose', turtlesim.msg.Pose, self.handle_turtle_pose)

        self.action_space = [i for i in range(self.action_size + 1)]
        self.q_table = np.array([[[0.0 for i in range(self.action_size + 1)] 
                         for j in range(int(self.max_distance_threshold / self.distance_step_size) + 1)] 
                         for k in range(int(360 / self.angle_step_size))]) # Q-table [angle][distance][action]


    def handle_turtle_pose(self, msg):
        if msg.linear_velocity == 0.0:
            self.is_reached = True
        return super(Q_Escaper, self).handle_turtle_pose(msg)
    
    def execute_vel_cmd(self, vel):
        # type: (geometry_msgs.msg.Twist) -> None
        self.is_reached = False
        self.pub_turtle_vel.publish(vel)
        self.rate.sleep()
    
    def get_q_value(self, angle_bin, distance_bin):
        # type: (int, int) -> np.array
        # Return the Q values of the given state
        # Note that the index is the action and the value is the Q value
        return self.q_table[angle_bin][distance_bin]

    def choose_action(self, angle_bin, distance_bin):
        # type: (int, int) -> int
        # Choose an action using epsilon-greedy algorithm
        if random.random() < self.epsilon:
            return random.choice(self.action_space)
        else:
            return self.get_q_value(angle_bin, distance_bin).argmax()

    def take_action(self, action):
        # type: (int) -> None
        # Take an action and execute
        print("Action: " + str(action))
        vel = geometry_msgs.msg.Twist()
        if action == 0:
            vel.linear.x = 0.0
            vel.linear.y = 0.0
        else:
            vel_angle = (action - 1) * self.angle_step_size * math.pi / 180
            vel.linear.x = self.max_moving_distance * math.cos(vel_angle)
            vel.linear.y = self.max_moving_distance * math.sin(vel_angle)

        self.execute_vel_cmd(vel)
   
    def update_q_table(self, angle_bin, distance_bin, action, reward, new_angle_bin, new_distance_bin):
        # type: (int, int, int, float, int, int) -> None
        # Update the Q-value using the Q-learning algorithm
        self.q_table[angle_bin][distance_bin][action] = (1 - self.alpha) * self.q_table[angle_bin][distance_bin][action] + self.alpha * (reward + self.gamma * self.get_q_value(new_angle_bin, new_distance_bin).max())

    