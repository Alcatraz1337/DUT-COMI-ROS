#!/usr/bin/env python

import rospy
import turtlesim.msg
import turtlesim.srv
import sys
import math
import random
import geometry_msgs.msg
from turtle_PSO_catcher import PSO_Catcher


class PSOManager:

    def __init__(self, pop, max_steps):
        rospy.init_node('pso_manager')

        self.population = pop
        self.MAX_STEPS = max_steps
        self.rate = rospy.Rate(10)
        self.gbest_pose = turtlesim.msg.Pose()
        self.gbest_value = sys.maxsize
        self.catchers = []  # List of catchers
        self.pbest_pose_list = []  # List of personal best poses
        self.inertia_weight = 0.5
        self.c1 = 2.0
        self.c2 = 2.0
        self.target_pose = turtlesim.msg.Pose()
        self.sub_target_pose = rospy.Subscriber('/turtle1/pose',
                                                turtlesim.msg.Pose,
                                                self.handle_target_pose)
        self.pub_target_pose_list = []

    def initialize(self):
        for i in range(self.population):
            self.catchers.append(PSO_Catcher(i + 2))
        for i in range(self.population):
            self.pub_target_pose_list.append(
                rospy.Publisher(self.catchers[i].name + '/target_pose',
                                turtlesim.msg.Pose,
                                queue_size=10,
                                latch=True))
        rospy.wait_for_service('spawn')
        for catcher in self.catchers:
            try:
                catcher.spawn()
            except Exception as e:
                rospy.logerr(e)
        # Initialize personal best list with the initial pose of the catchers and a large value
        for i in range(self.population):
            self.pbest_pose_list.append((turtlesim.msg.Pose(), sys.maxsize))

    def handle_target_pose(self, msg):
        self.target_pose = msg

    def calculate_fitness(self, catcher_pose, target_pose):
        return math.sqrt((catcher_pose.x - target_pose.x)**2 +
                         (catcher_pose.y - target_pose.y)**2)

    def update_pbest_list(self):
        for i in range(self.population):
            fitness = self.calculate_fitness(self.catchers[i].pose,
                                             self.target_pose)
            if fitness < self.pbest_pose_list[i][1]:
                self.pbest_pose_list[i] = (self.catchers[i].pose, fitness)

    def find_global_best(self):
        self.update_pbest_list()
        for i in range(self.population):
            if self.pbest_pose_list[i][1] < self.gbest_value:
                self.gbest_pose = self.pbest_pose_list[i][0]
                self.gbest_value = self.pbest_pose_list[i][1]

    def generate_random_velocity(self, catcher_index):
        random_distance = random.uniform(
            0.0, self.catchers[catcher_index].max_moving_distance)
        random_angle = random.uniform(0.0, 2 * math.pi)

        random_velocity = geometry_msgs.msg.Twist()
        random_velocity.linear.x = random_distance * math.cos(random_angle)
        random_velocity.linear.y = random_distance * math.sin(random_angle)

        return random_velocity

    def generate_adjusted_velocity(self, catcher_index):
        adjusted_velocity = geometry_msgs.msg.Twist()

        dx_gbest = self.gbest_pose.x - self.catchers[catcher_index].pose.x
        dy_gbest = self.gbest_pose.y - self.catchers[catcher_index].pose.y

        dx_pbest = self.pbest_pose_list[catcher_index][0].x - self.catchers[
            catcher_index].pose.x
        dy_pbest = self.pbest_pose_list[catcher_index][0].y - self.catchers[
            catcher_index].pose.y

        rand_c1 = random.random()
        rand_c2 = random.random()

        dx_new = self.inertia_weight * self.catchers[
            catcher_index].last_vel.linear.x + rand_c1 * self.c1 * dx_pbest + rand_c2 * self.c2 * dx_gbest
        dy_new = self.inertia_weight * self.catchers[
            catcher_index].last_vel.linear.y + rand_c1 * self.c1 * dy_pbest + rand_c2 * self.c2 * dy_gbest

        distance_new = math.sqrt(dx_new**2 + dy_new**2)
        print("distance_new: %f" % distance_new)
        if distance_new < self.catchers[catcher_index].max_moving_distance:
            adjusted_velocity.linear.x = dx_new
            adjusted_velocity.linear.y = dy_new
            return adjusted_velocity
        else:
            # Adjust the new velocity to the maximum moving distance
            theta = math.atan2(dy_new, dx_new)
            adjusted_velocity.linear.x = self.catchers[
                catcher_index].max_moving_distance * math.cos(theta)
            adjusted_velocity.linear.y = self.catchers[
                catcher_index].max_moving_distance * math.sin(theta)
            return adjusted_velocity

    def generate_adjusted_pose(self, catcher_index):
        adjusted_pose = turtlesim.msg.Pose()

        dx_gbest = self.gbest_pose.x - self.catchers[catcher_index].pose.x
        dy_gbest = self.gbest_pose.y - self.catchers[catcher_index].pose.y

        dx_pbest = self.pbest_pose_list[catcher_index][0].x - self.catchers[
            catcher_index].pose.x
        dy_pbest = self.pbest_pose_list[catcher_index][0].y - self.catchers[
            catcher_index].pose.y

        rand_c1 = random.random()
        rand_c2 = random.random()

        dx_new = rand_c1 * self.c1 * dx_pbest + rand_c2 * self.c2 * dx_gbest
        dy_new = rand_c1 * self.c1 * dy_pbest + rand_c2 * self.c2 * dy_gbest

        distance_new = math.sqrt(dx_new**2 + dy_new**2)
        print("distance_new: %f" % distance_new)
        if distance_new < self.catchers[catcher_index].max_moving_distance:
            adjusted_pose.x = self.catchers[catcher_index].pose.x + dx_new
            adjusted_pose.y = self.catchers[catcher_index].pose.y + dy_new
            return adjusted_pose
        else:
            # Adjust the new pose to the maximum moving distance
            theta = math.atan2(dy_new, dx_new)
            adjusted_pose.x = self.catchers[
                catcher_index].pose.x + self.catchers[
                    catcher_index].max_moving_distance * math.cos(theta)
            adjusted_pose.y = self.catchers[
                catcher_index].pose.y + self.catchers[
                    catcher_index].max_moving_distance * math.sin(theta)
            return adjusted_pose

    def generate_random_pose(self, cathcer_index):
        # Generate random pose for each catcher within its own maximum moving distance
        random_pose = turtlesim.msg.Pose()
        random_angle = random.uniform(0, 2 * math.pi)
        random_distance = random.uniform(
            0, self.catchers[cathcer_index].max_moving_distance)
        random_pose.x = self.catchers[
            cathcer_index].pose.x + random_distance * math.cos(random_angle)
        random_pose.y = self.catchers[
            cathcer_index].pose.y + random_distance * math.sin(random_angle)
        return random_pose

    def run(self):
        step = 0
        while not rospy.is_shutdown() and step < self.MAX_STEPS:
            self.rate.sleep()
            if step == 0:
                for i in range(self.population):
                    # Generate a random velocity for each catcher
                    random_velocity = self.generate_random_velocity(i)
                    self.catchers[i].last_vel = random_velocity
                    self.catchers[i].is_reached = False
                    self.catchers[i].execute_vel_cmd(random_velocity)
                self.rate.sleep()
            else:
                for i in range(self.population):
                    # Generate an adjusted velocity for each catcher
                    adjusted_velocity = self.generate_adjusted_velocity(i)
                    self.catchers[i].last_vel = adjusted_velocity
                    self.catchers[i].is_reached = False
                    self.catchers[i].execute_vel_cmd(adjusted_velocity)
                self.rate.sleep()

            print("Finding global best...")
            self.find_global_best()
            print("Global best: %s" % self.gbest_pose)

            # Wait until all catchers reach the target
            all_set = False
            while not all_set:
                all_set = True
                for i in range(self.population):
                    if not self.catchers[i].is_reached:
                        all_set = False
                        break
                self.rate.sleep()

            step += 1
            self.rate.sleep()

    def exit(self):
        rospy.wait_for_service('kill')
        kill = rospy.ServiceProxy('kill', turtlesim.srv.Kill)
        for catcher in self.catchers:
            try:
                kill(catcher.name)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
