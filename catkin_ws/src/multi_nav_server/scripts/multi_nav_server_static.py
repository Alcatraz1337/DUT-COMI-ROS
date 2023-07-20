#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID
from station import Station, Stations
from worker import Worker
from car_status_msgs.msg import CarStatus
from arm_status_msgs.msg import ArmStatus
from jobs import Jobs
import sys


class MultiNavServer:
    def __init__(self, n_cars=1):
        self._rate = rospy.Rate(10)
        self._stations = [] # type: list[Station]
        self._cars = []  # type: list[Worker]
        # self._point_index = 0  # Probably will discard this
        self._n_cars = n_cars
        self._n_stations = 0
        self.sub_car_status = rospy.Subscriber('/car_status', CarStatus, self.car_status_callback)
        self.sub_arm_status = rospy.Subscriber('/arm_status', ArmStatus, self.arm_status_callback)

        self._available_cars = []  # type:list[int] # list of available car ids, id is the index of the car in the cars list, [0, n_cars]
        self._working_cars = []  # type:list[int] # list of working car ids, id is the index of the car in the cars list [0, n_cars]
        self._available_stations = []  # type:list[int] # list of available station indices, [0, n_stations]
        self._working_stations = []  # type:list[int] # list of working station indices, [0, n_stations]
        self._jobs = {}  # type: dict[str, dict] # All changes update to this dict
        self._available_jobs = [] # type: list[str]
        self._working_jobs = [] # type: list[str]

    def initialize(self):
        self._stations = Stations().get_stations()
        # Initialize cars indices
        if self._n_cars == 1:
            self._cars.append(Worker("", 0))
            self._available_cars.append(0)
        else:
            for i in range(self._n_cars):
                # TODO: Set to right car name
                worker = Worker("car" + str(i), i)
                self._cars.append(worker)
                self._available_cars.append(i)
        self._n_stations = len(self._stations)
        # Initialize static stations and set station arm id. And add available stations
        for i in range(self._n_stations):
            self._stations[i]._id = self._n_cars + i # station id starts from n_cars
            if i != 0:
                self._available_stations.append(i)
        # Initialize jobs
        self._jobs = Jobs().get_jobs()
        for job in self._jobs.items():
            self._available_jobs.append(job[0])

    def shutdown(self):
        self._cars[0].shutdown()
        # self.pub_markers.unregister()
        # self.pub_goal.unregister()
        # self.sub_goal_result.unregister()
        rospy.loginfo("Shutting down multi_nav_server")

    def car_status_callback(self, msg):
        # type: (CarStatus) -> None
        if msg.car_ready:
            rospy.loginfo("Car " + str(msg.id) + " is ready")
            self._available_cars.append(int(msg.id))
            # The car finished transporting the object, then add the station to working stations list
            self._working_stations.append(self._jobs[msg.job]["station"])

    def arm_status_callback(self, msg):
        # type: (ArmStatus) -> None
        # If a station arm finished working
        if msg.status and msg.arm_id >= self._n_cars:
            rospy.loginfo("Server: Arm " + str(msg.arm_id) + " finished working")
            self._available_jobs.append(msg.job)
            self._available_stations.append(msg.arm_id - self._n_cars)
            self._working_stations.remove(msg.arm_id - self._n_cars)
            self._stations[msg.arm_id - self._n_cars].is_working = False

    def get_dispatch_routes(self, job):
        # type: (str) -> tuple[int, int]
        """
        Get a job and return the start and end point indices
        First get the job's currently working station index
        Second get the job's next station index from available stations
        """
        start = self._jobs[job]["station"]
        # if this is the last process of the job then go to distribution station
        end = 0 if self._jobs[job]["process"] == 1 else self._available_stations.pop(0)
        return start, end
    
    def start_stations(self):
        # type: () -> None
        """
        Check all stations in the working_stations list, if a station is not working, start it
        """
        for station_index in self._working_stations:
            if not self._stations[station_index].is_working:
                self._stations[station_index].is_working = True
                self._stations[station_index].start_arm()
    
    def has_available_jobs(self):
        # type: () -> bool
        return len(self._available_jobs) > 0
    
    def has_available_cars(self):
        # type: () -> bool
        return len(self._available_cars) > 0
    
    def all_job_done(self):
        # type: () -> bool
        return len(self._available_jobs) == 0 and len(self._working_jobs) == 0

    def dispatch_car(self):
        # type:() -> None
        """
        car_index: int, index of the car in the cars list
        point_index: int, index of the point in the points list
        mission_type: str, "pick" or "drop"
        obj: str, optional,

        All available/working cars/stations should be handled here
        Also should announce the station what job it will be working on
        """
        # Get job and car index using FIFO greedy algorithm
        job = self._available_jobs.pop(0)
        car_index = self._available_cars.pop(0)
        route = self.get_dispatch_routes(job)
        # Check car_index and point_index
        if car_index < 0 or car_index >= len(self._cars):
            rospy.logwarn("Invalid car index")
            return
        if route[0] < 0 or route[0] >= len(self._stations):
            rospy.logwarn("Invalid point index")
            return
        # dispatch car to point and set mission object and set target indices
        self._cars[car_index].set_working_obj_color(self._jobs[job]["target_color"])
        self._cars[car_index].set_moving_targets(route[0], route[1])
        self._cars[car_index].activate_car()
        # Set the station's job that it will be doing. The station should be the route's end point
        self._stations[route[1]].set_working_color(self._jobs[job]["target_color"])
        # Add the car to working_car list
        self._working_cars.append(car_index)
        # Add the job to working_job list
        self._working_jobs.append(job)

        self._jobs[job]["process"] -= 1 # Decrease the job's process by 1
        self._jobs[job]["station"] = route[1] # Set the job's station to the next station
        self._available_stations.remove(route[1]) # The next station is not available anymore

    def run(self):
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self._rate.sleep()
            if not self.all_job_done():
                if self.has_available_cars() and self.has_available_jobs():
                    self.dispatch_car()
                    self.start_stations()
            else:
                rospy.loginfo("All jobs done, shutting down...")
                self._rate.sleep()
                break
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
