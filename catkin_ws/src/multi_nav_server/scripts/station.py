#!/usr/bin/env python2

import rospy
from move_base_msgs.msg import MoveBaseGoal
from arm_work_msgs.msg import ArmWork


class Station:
    def __init__(self, id=0):
        self._id = id
        # Initialize a list called points, with given PoseStamped arguments.
        self.input = MoveBaseGoal()
        self.output = MoveBaseGoal()
        # self.pub_arm_status = rospy.Publisher('/arm_status', ArmStatus, queue_size=1)
        self.pub_arm_work = rospy.Publisher('/arm_work', ArmWork, queue_size=1)
        self.is_working = False
        self.occupied_picking = False  # type: bool # Is the station occupied by a worker picking up an object?
        self._color = ""  # type: str # Color of the object
        self._job = ""  # type: str # Job of the object

    def set_job(self, job):
        # type: (str) -> None
        self._job = job

    def set_working_color(self, color):
        # type: (str) -> None
        self._color = color

    def start_arm(self):
        # type: () -> None
        self.pub_arm_work.publish(ArmWork(self._id, "pick", self._job, self._color))


class Stations:
    def __init__(self):
        self.stations = []

        # These 4 stations are for testing purposes only.
        # The first is for distribution station, the rest are for working stations.
        # TODO: Change the coordinates to the actual stations.
        st = Station()
        st.input.target_pose.pose.position.x = -1.5
        st.input.target_pose.pose.position.y = 0.5
        st.input.target_pose.pose.orientation.z = 0.0
        st.input.target_pose.pose.orientation.w = 1.0
        st.output.target_pose.pose.position.x = -1.0
        st.output.target_pose.pose.position.y = 1.0
        st.output.target_pose.pose.orientation.z = -0.7
        st.output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

        st = Station()
        st.input.target_pose.pose.position.x = -1.35
        st.input.target_pose.pose.position.y = 4.0
        st.input.target_pose.pose.orientation.z = 0.0
        st.input.target_pose.pose.orientation.w = 1.0
        st.output.target_pose.pose.position.x = -1.0
        st.output.target_pose.pose.position.y = 3.7
        st.output.target_pose.pose.orientation.z = 0.7
        st.output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

        st = Station()
        st.input.target_pose.pose.position.x = -3.7
        st.input.target_pose.pose.position.y = 4.0
        st.input.target_pose.pose.orientation.z = 1.0
        st.input.target_pose.pose.orientation.w = 0.0
        st.output.target_pose.pose.position.x = -4.0
        st.output.target_pose.pose.position.y = 3.7
        st.output.target_pose.pose.orientation.z = 0.7
        st.output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

        st = Station()
        st.input.target_pose.pose.position.x = -3.5
        st.input.target_pose.pose.position.y = 1.0
        st.input.target_pose.pose.orientation.z = 1.0
        st.input.target_pose.pose.orientation.w = 0.0
        st.output.target_pose.pose.position.x = -4.0
        st.output.target_pose.pose.position.y = 1.3
        st.output.target_pose.pose.orientation.z = -0.7
        st.output.target_pose.pose.orientation.w = 0.7
        self.stations.append(st)

    def get_stations(self):
        # type: () -> list[Station]
        return self.stations
