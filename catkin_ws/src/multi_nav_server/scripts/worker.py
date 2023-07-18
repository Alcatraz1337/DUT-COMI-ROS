#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID
from arm_status_msgs.msg import ArmStatus
from car_status_msgs.msg import CarStatus
import actionlib
from station import Stations
from Arm_util.arm_car import Arm_car


class Worker:
    def __init__(self, name="", id=0):
        self._name = name
        self._id = id
        self._stations = Stations().get_stations()
        self.sub_goal_result = rospy.Subscriber(self._name + '/move_base/result', MoveBaseActionResult,
                                                self.goal_result_callback)
        self.sub_arm_status = rospy.Subscriber(self._name + '/arm_status', ArmStatus, self.arm_status_callback)
        self.pub_cmd_vel = rospy.Publisher(self._name + '/cmd_vel', Twist, queue_size=1)
        self.pub_cancel_goal = rospy.Publisher(self._name + '/move_base/cancel', GoalID, queue_size=1)
        self.pub_car_status = rospy.Publisher('/car_status', CarStatus, queue_size=1)
        self.move_base_client = actionlib.SimpleActionClient(self._name + '/move_base', MoveBaseAction)
        # TODO: Create Arm class
        self._arm = Arm_car(self._id, 3)

        self._is_ready = True
        self._is_moving = False
        self._arm_picking = False
        self._arm_dropping = False
        self._arm_working = False
        self._arm_obj = ""  # Working object
        self._curr_target = -1  # Current target index
        self._next_target = -1  # Next target index

    def shutdown(self):
        self.pub_cancel_goal.publish(GoalID())
        self.pub_cmd_vel.publish(Twist())
        self.sub_goal_result.unregister()
        self.sub_arm_status.unregister()
        rospy.loginfo("Shutting down worker " + self._name)

    def set_arm_id(self, id):
        self._arm.set_id(id)

    def set_object(self, obj):
        # type: (str) -> None
        self._arm_obj = obj
        self._arm_picking = True
        self._arm_dropping = True

    def set_moving_targets(self, curr_target, next_target):
        # type: (int, int) -> None
        if curr_target < 0 or curr_target >= len(self._stations):
            rospy.logwarn("Invalid current target index")
            self._curr_target = -1
        else:
            self._curr_target = curr_target
        if next_target < 0 or next_target >= len(self._stations):
            rospy.logwarn("Invalid next target index")
            self._next_target = -1
        else:
            self._next_target = next_target

    def activate_car(self):
        # type: () -> None
        # Error handling
        if self._curr_target < 0 or self._curr_target >= len(self._stations):
            rospy.logwarn("Invalid current target index")
            return
        if self._is_moving:
            rospy.logwarn("Car " + str(self._id) + " is already moving")
            return
        # Send goal
        goal = MoveBaseGoal()
        # TODO: Check if all the situation is go to output first then input
        goal = self._stations[self._curr_target]._output if self._next_target != -1 \
                                                        else self._stations[self._curr_target]._input
        goal.target_pose.header.frame_id = "map"

        self.move_base_client.send_goal(goal)
        rospy.loginfo("Activate: Sending goal " + str(self._curr_target) + " to car " + str(self._id))
        self._curr_target = self._next_target
        self._next_target = -1
        self._is_ready = False
        self._is_moving = True

    def arm_pick(self, obj=""):
        # type: (str) -> None
        # TODO: Set arm to pick specific object
        # self.Arm.arm_pick(obj)
        self._arm_working = True
        pass

    def arm_drop(self):
        # TODO: Set arm to drop the object
        self._arm_working = True
        pass

    def goal_result_callback(self, msg):
        # type: (MoveBaseActionResult) -> None
        if msg.status.status == 3:
            rospy.loginfo("Goal reached, setting car " + str(self._id) + " not moving, continuing to arm...")
            self._is_moving = False
            if self._arm_obj != "":
                self.arm_pick(self._arm_obj)
            else:
                self._is_ready = True
                # Publish car status
                self.pub_car_status.publish(CarStatus(self._id, self._is_moving, self._is_ready))

    def arm_status_callback(self, msg):
        # type: (ArmStatus) -> None
        # TODO: When arm finished moving, update arm status
        if msg.arm_id == self._id and msg.status:
            rospy.loginfo("Car " + str(self._id) + " arm finished!")
            if self._is_moving:
                # If the car is still moving, the arm should not be working..
                # TODO: Fix the problem if the arm is working when the car is moving
                return
            # The first call should be the arm finished picking, so the next move is to go the next target
            if self._arm_picking:
                rospy.loginfo(
                    "Car " + str(self._id) + " finished picking, sending goal " + str(self._curr_target) + "...")
                self._arm_picking = False
                self.activate_car()
            # The second call should be the arm finished dropping, so the next move is set the car to ready
            elif self._arm_dropping:
                rospy.loginfo("Car " + str(self._id) + " finished dropping, setting car to ready...")
                self._arm_dropping = False
                self._arm_working = False
                self._is_ready = True
                # Publish car status
                self.pub_car_status.publish(CarStatus(self._id, self._is_moving, self._is_ready))
