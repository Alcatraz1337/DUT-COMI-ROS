#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID
import actionlib
from multi_nav_server.scripts.station import Stations
from worker import Worker
from car_status_msgs.msg import CarStatus
import sys


class MultiNavServer:
    def __init__(self, _n_cars=1):
        self._rate = rospy.Rate(10)
        self.stations = []
        self.point_index = 0  # Probably will discard this
        self.n_stations = 0
        self.n_cars = _n_cars
        self.cars = []  # type: list[Worker]
        self.stations = []  # list of station indices
        self.sub_car_status = rospy.Subscriber('/car_status', CarStatus, self.car_status_callback)
        self.available_cars = []  # list of available car indices

    def initialize(self):
        self.stations = Stations().get_points()
        if self.n_cars == 1:
            self.cars.append(Worker("", 0))
            self.available_cars.append(0)
        else:
            for i in range(self.n_cars):
                worker = Worker("car" + str(i), i)
                self.cars.append(worker)
                self.available_cars.append(i)
        self.n_stations = len(self.stations)
        for i in range(self.n_stations):
            self.stations[i].id = i

    def shutdown(self):
        self.cars[0].shutdown()
        # self.pub_markers.unregister()
        # self.pub_goal.unregister()
        # self.sub_goal_result.unregister()
        rospy.loginfo("Shutting down multi_nav_server")

    def car_status_callback(self, msg):
        if msg.car_ready:
            rospy.loginfo("Car " + str(msg.id) + " is ready")
            self.available_cars.append(int(msg.id))

    """
    def goal_result_callback(self, msg):
    Has been moved to worker.py and send_goal() method of this class has been deprecated.
    
    def send_goal(self, car_index, point_index):
        # Check if point index is valid
        if point_index < 0 or point_index >= len(self.points):
            rospy.logwarn("Invalid point index")
            return

        # Check if car index is valid
        if car_index < 0 or car_index >= len(self.cars):
            rospy.logwarn("Invalid car index")
            return

        # Send goal via SimpleActionClient
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = self.points[point_index].target_pose.pose.position.x
        goal.target_pose.pose.position.y = self.points[point_index].target_pose.pose.position.y
        goal.target_pose.pose.orientation.z = self.points[point_index].target_pose.pose.orientation.z
        goal.target_pose.pose.orientation.w = self.points[point_index].target_pose.pose.orientation.w
        self.cars[car_index].move_base_client.send_goal(goal)
        rospy.loginfo("Sending goal to car " + str(car_index) + "...")
        self.cars[car_index].is_ready = False
        self.cars[car_index].is_moving = True
    """

    def dispatch_car(self, car_index, start_point_index, end_point_index, obj=""):
        # type:(int, int, int, str) -> None
        """
        car_index: int, index of the car in the cars list
        point_index: int, index of the point in the points list
        mission_type: str, "pick" or "drop"
        obj: str, optional,
        """
        # check car_index, point_index and mission_type validity
        if car_index < 0 or car_index >= len(self.cars):
            rospy.logwarn("Invalid car index")
            return
        if start_point_index < 0 or start_point_index >= len(self.stations):
            rospy.logwarn("Invalid point index")
            return
        # dispatch car to point and set mission object and set target indices
        self.cars[car_index].set_object(obj)
        self.cars[car_index].set_targets(start_point_index, end_point_index)
        self.cars[car_index].activate_car()
        self.available_cars.remove(car_index)

    def run(self):
        rospy.on_shutdown(self.shutdown)
        self.cars[0].move_base_client.wait_for_server()
        while not rospy.is_shutdown():
            self._rate.sleep()
            if len(self.available_cars) > 0:
                if self.point_index >= len(self.stations):
                    rospy.loginfo("All stations reached, shutting down...")
                    self._rate.sleep()
                    break
                else:
                    self.dispatch_car(0, self.point_index, self.point_index + 1, "red")
                    self.point_index += 2
            self._rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('multi_nav_server_static')
        args = rospy.myargv(argv=sys.argv)
        multi_nav_server = MultiNavServer(int(args[1]))
        multi_nav_server.initialize()
        multi_nav_server.run()
    except rospy.ROSInterruptException:
        pass
