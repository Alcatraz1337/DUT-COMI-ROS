#!/usr/bin/env python

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseActionResult

rospy.init_node('send_client_goal')

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
is_car_ready = True
goal_index = 0

goal1 = MoveBaseGoal()
goal1.target_pose.header.frame_id = 'map'
goal1.target_pose.pose.position.x = -1.0
goal1.target_pose.pose.position.y = 4.0
goal1.target_pose.pose.orientation.z = 0.7
goal1.target_pose.pose.orientation.w = 0.7

goal2 = MoveBaseGoal()
goal2.target_pose.header.frame_id = 'map'
goal2.target_pose.pose.position.x = -4.0
goal2.target_pose.pose.position.y = 4.0
goal2.target_pose.pose.orientation.z = 1.0
goal2.target_pose.pose.orientation.w = 0.0

goals = [goal1, goal2]

def goal_result_callback(msg):
    print("Goal reached, setting car ready...")
    global is_car_ready
    is_car_ready = True

rospy.loginfo("Waiting for move base server")
client.wait_for_server()
sub_result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_result_callback)

while not rospy.is_shutdown():
    if is_car_ready:
        if goal_index < len(goals):
            goal = MoveBaseGoal()
            goal = goals[goal_index]
            rospy.loginfo("Sending goal " + str(goal_index) + " to move base server")
            client.send_goal(goal)
            is_car_ready = False
            goal_index += 1
        else:
            rospy.loginfo("All goals reached, shutting down...")
            break

