#!/usr/bin/env python2

import rospy, sys, os
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID
from arm_status_msgs.msg import ArmStatus
from arm_work_msgs.msg import ArmWork
from car_status_msgs.msg import CarStatus
import actionlib
from station import Stations


class Worker:
    def __init__(self, name="", id=0):
        self._name = name
        self._id = id
        self._stations = Stations().get_stations()
        self.sub_goal_result = rospy.Subscriber(self._name + '/move_base/result', MoveBaseActionResult,
                                                self.goal_result_callback)
        # self.sub_arm_status = rospy.Subscriber('/arm_status', ArmStatus, self.arm_status_callback) # Deprecated
        self.pub_cmd_vel = rospy.Publisher(self._name + '/cmd_vel', Twist, queue_size=1)
        self.pub_cancel_goal = rospy.Publisher(self._name + '/move_base/cancel', GoalID, queue_size=1)
        # self.pub_car_status = rospy.Publisher('/car_status', CarStatus, queue_size=1) # Deprecated
        self.move_base_client = actionlib.SimpleActionClient(self._name + '/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.pub_arm_work = rospy.Publisher('/arm_work', ArmWork, queue_size=1)

        self.is_ready = True
        self.is_moving = False
        self.arm_picking = False
        self.arm_dropping = False
        self.arm_working = False
        self._job = ""  # Current job
        self._arm_obj_color = ""  # Working object
        self._curr_target = -1  # Current target index
        self._next_target = -1  # Next target index

    def shutdown(self):
        self.pub_cancel_goal.publish(GoalID())
        self.pub_cmd_vel.publish(Twist())
        self.sub_goal_result.unregister()
        rospy.loginfo("Shutting down worker " + self._name)

    # Deprecated due to the use of ArmWork msgs
    # def set_arm_id(self, id):
    #     self._arm.set_id(id)

    def set_working_job_color(self, job, color):
        # type: (str, str) -> None
        # Set the car's target object and mark both signs to true
        self._job = job
        self._arm_obj_color = color
        self.arm_picking = True
        self.arm_dropping = True

    def set_moving_targets(self, curr_target, next_target):
        # type: (int, int) -> None
        # Check if the targets are valid
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
            rospy.logwarn("[Worker {}]: Invalid current target index".format(self._id))
            return
        if self.is_moving:
            rospy.logwarn("[Worker " + str(self._id) + "] is already moving")
            return
        # Send goal
        goal = MoveBaseGoal()
        # All the situation is go to output first then input
        # if the next target is -1, then should go to input
        goal = self._stations[self._curr_target].output if self._next_target != -1 \
            else self._stations[self._curr_target].input
        goal.target_pose.header.frame_id = "map"
        self.move_base_client.send_goal(goal)
        rospy.loginfo("Activate: Sending goal " + str(self._curr_target) + " to car " + str(self._id))
        # Move next target to current target
        self._curr_target = self._next_target
        self._next_target = -1
        self.is_ready = False
        self.is_moving = True

    def arm_pick(self, obj=""):
        # type: (str) -> None
        # TODO: Set arm to pick specific object
        msg = ArmWork(self._id, "pick", self._job, obj)
        self.pub_arm_work.publish(msg)
        self.arm_working = True

    def arm_drop(self):
        # TODO: Set arm to drop the object
        msg = ArmWork(self._id, "drop", self._job, "")
        self.pub_arm_work.publish(msg)
        self.arm_working = True

    def goal_result_callback(self, msg):
        # type: (MoveBaseActionResult) -> None
        if not isinstance(msg, MoveBaseActionResult): return
        if msg.status.status == 3:
            rospy.loginfo("Goal reached, setting car " + str(self._id) + " not moving, continuing to arm...")
            self.is_moving = False
            if self.arm_picking:
                # If the car has an object to pick, then pick it
                self.arm_pick(self._arm_obj_color)
            elif self.arm_dropping:
                # If the car has no object to pick, then drop the object
                self.arm_drop()

    """
    The following code is deprecated due to all arm related functions are handled in the server
    def arm_status_callback(self, msg):
        # type: (ArmStatus) -> None
        # TODO: When arm finished moving, update arm status
        if msg.arm_id == self._id and msg.status:
            rospy.loginfo("Car " + str(self._id) + " arm finished!")
            if self.is_moving:
                # If the car is still moving, the arm should not be working..
                # TODO: Fix the problem if the arm is working when the car is moving
                rospy.logwarn("Car " + str(self._id) + " is moving, but arm is working, this should not happen!")
                return
            # The first call should be the arm finished picking, so the next move is to go the next target
            if self.arm_picking:
                rospy.loginfo(
                    "Car " + str(self._id) + " finished picking, sending goal " + str(self.curr_target) + "...")
                self.arm_picking = False
                self.activate_car()
            # The second call should be the arm finished dropping, so the next move is set the car to ready
            elif self.arm_dropping:
                rospy.loginfo("Car " + str(self._id) + " finished dropping, setting car to ready...")
                self.arm_dropping = False
                self.arm_working = False
                self.is_ready = True
                # Publish car status
                self.pub_car_status.publish(CarStatus(self._id, self._job, self.is_ready))

    """
