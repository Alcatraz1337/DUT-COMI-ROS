#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal
from arm_status_msgs.msg import ArmStatus
from Arm_util.arm_static import Arm_static

class Station:
    def __init__(self, id=0):
        self._id = id
        # Initialize a list called points, with given PoseStamped arguments.
        self._input = MoveBaseGoal()
        self._output = MoveBaseGoal()
        self.pub_arm_status = rospy.Publisher('/arm_status', ArmStatus, queue_size=1)
        self._arm = Arm_static(self._id, 3)

    def set_arm_id(self, id):
        self._arm.set_id(id)


class Stations():
    def __init__(self):
        self.stations = []

        # These 4 stations are for testing purposes only.
        # The first is for distribution station, the rest are for working stations.
        # TODO: Change the coordinates to the actual stations.
        st = Station()
        st._input.target_pose.pose.position.x = -1.5
        st._input.target_pose.pose.position.y = 0.5
        st._input.target_pose.pose.orientation.z = 0.0
        st._input.target_pose.pose.orientation.w = 1.0
        st._output.target_pose.pose.position.x = -1.0
        st._output.target_pose.pose.position.y = 1.0
        st._output.target_pose.pose.orientation.z = -0.7
        st._output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

        st = Station()
        st._input.target_pose.pose.position.x = -1.35
        st._input.target_pose.pose.position.y = 4.0
        st._input.target_pose.pose.orientation.z = 0.0
        st._input.target_pose.pose.orientation.w = 1.0
        st._output.target_pose.pose.position.x = -1.0
        st._output.target_pose.pose.position.y = 3.7
        st._output.target_pose.pose.orientation.z = 0.7
        st._output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

        st = Station()
        st._input.target_pose.pose.position.x = -3.7
        st._input.target_pose.pose.position.y = 4.0
        st._input.target_pose.pose.orientation.z = 1.0
        st._input.target_pose.pose.orientation.w = 0.0
        st._output.target_pose.pose.position.x = -4.0
        st._output.target_pose.pose.position.y = 3.7
        st._output.target_pose.pose.orientation.z = 0.7
        st._output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

        st = Station()
        st._input.target_pose.pose.position.x = -3.5
        st._input.target_pose.pose.position.y = 1.0
        st._input.target_pose.pose.orientation.z = 1.0
        st._input.target_pose.pose.orientation.w = 0.0
        st._output.target_pose.pose.position.x = -4.0
        st._output.target_pose.pose.position.y = 1.3
        st._output.target_pose.pose.orientation.z = -0.7
        st._output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

    def get_stations(self):
        # type: () -> list[Station]
        return self.stations