#!/usr/bin/env python2

import rospy
from station import Station, Stations
from worker import Worker
from arm_status_msgs.msg import ArmStatus
from jobs import Jobs
import sys


class MultiNavServer:
    def __init__(self, n_cars=1):
        self._rate = rospy.Rate(10)
        self._stations = []  # type: list[Station]
        self._cars = []  # type: list[Worker]
        self._n_cars = n_cars
        self._n_stations = 0
        # self.sub_car_status = rospy.Subscriber('/car_status', CarStatus, self.car_status_callback) # Deprecated
        self.sub_arm_status = rospy.Subscriber('/arm_status', ArmStatus, self.arm_status_callback)

        self._available_cars_id = []  # type:list[int] # list of available car ids, id is the index of the car in the cars list + 1, [1, n_cars]
        self._working_cars_id = []  # type:list[int] # list of working car ids, id is the index of the car in the cars list + 1 [1, n_cars]
        self._available_stations_idx = []  # type:list[int] # list of available station indices, [0, n_stations - 1]
        self._working_stations_idx = []  # type:list[int] # list of working station indices, [0, n_stations - 1]
        self._jobs = {}  # type: dict[str, dict] # All changes update to this dict
        self._available_jobs_name = []  # type: list[str]
        self._working_jobs_name = []  # type: list[str]

    def initialize(self):
        self._stations = Stations().get_stations()
        # Initialize cars indices
        if self._n_cars == 1:
            self._cars.append(Worker("", 0))
            self._available_cars_id.append(0)
        else:
            for i in range(self._n_cars):
                # Robot name starts from robot1
                rospy.loginfo("[Server] Initializing car " + str(i + 1))
                worker = Worker("robot" + str(i + 1), i + 1)
                self._cars.append(worker)
                self._available_cars_id.append(i + 1)
        self._n_stations = len(self._stations)
        # Initialize static stations and set station arm id. And add available stations
        for i in range(self._n_stations):
            rospy.loginfo("[Server] Initializing station " + str(self._n_cars + i + 1))
            self._stations[i]._id = self._n_cars + i + 1 # station id starts from n_cars + 1
            if i != 0:  # The first station is the distribution station, so it should not be added to available stations
                self._available_stations_idx.append(i)
        # Initialize jobs
        rospy.loginfo("[Server] Initializing jobs...")
        self._jobs = Jobs().get_jobs()
        for job in self._jobs.items():
            self._available_jobs_name.append(job[0])
        rospy.loginfo("[Server] Done initializing server")

    def shutdown(self):
        for i in range(self._n_cars):
            self._cars[i].shutdown()
        # self.pub_markers.unregister()
        # self.pub_goal.unregister()
        # self.sub_goal_result.unregister()
        rospy.loginfo("[Server] Shutting down multi_nav_server")

    """
    # Probably deprecated. Due to the car would never publish car_ready
    # Car's arm is handle in the server, so the car's status is not needed
    def car_status_callback(self, msg):
        # type: (CarStatus) -> None
        if msg.car_ready:
            rospy.loginfo("[Server] Car " + str(msg.id) + " is ready")
            self._available_cars.append(int(msg.id))
            self._jobs[msg.job]["transfer"]["start"] = self._jobs[msg.job]["transfer"]["end"]
    """

    def print_available_jobs(self):
        # type: () -> None
        # Print all jobs in availble list
        jobs = ""
        for job in self._available_jobs_name:
            jobs = jobs + "{} (obj: {}), ".format(job, self._jobs[job]["target_color"])
        rospy.loginfo("Current available jobs: [{}]".format(jobs))

    def print_available_stations(self):
        # type: () -> None
        # Print all stations available
        stations = ""
        for station in self._available_stations_idx:
            stations = stations + "st: {}, ".format(station + 1 + self._n_cars)
        rospy.loginfo("Current available stations: [{}]".format(stations))
        
    def print_available_cars(self):
        # type: () -> None
        # Print all cars available
        cars = ""
        for car in self._available_cars_id:
            cars = cars + "car{}, ".format(car)
        rospy.loginfo("Current available cars: [{}]".format(cars))

    def print_all_available(self):
        # type: () -> None
        # Print all available cars / stations / jobs
        self.print_available_stations()
        self.print_available_jobs()
        self.print_available_cars()

    def arm_status_callback(self, msg):
        # type: (ArmStatus) -> None
        # If a station arm finished working
        if not isinstance(msg, ArmStatus): return
        if msg.status and msg.arm_id > self._n_cars:
            rospy.loginfo("[Server] Station {} finished working (obj: {})".format(msg.arm_id, self._jobs[msg.job]["target_color"]))
            # If a job has no more process, then don't need to add it to available jobs list
            if self._jobs[msg.job]["process"] >= 0:
                self._available_jobs_name.append(msg.job)
            self._working_jobs_name.remove(msg.job)
            self._available_stations_idx.append(msg.arm_id - self._n_cars - 1) # Note its the index
            self._working_stations_idx.remove(msg.arm_id - self._n_cars - 1)
            self._stations[msg.arm_id - self._n_cars - 1].is_working = False
            self.print_all_available()

            return  # Don't need to check the rest

        # If a car's arm finished working (picking or dropping)
        if msg.arm_id <= self._n_cars and msg.status:
            rospy.loginfo("[Server] Car {} arm finished!".format(msg.arm_id))
            if self._cars[msg.arm_id - 1].is_moving:
                # If the car is still moving, the arm should not be working..
                # TODO: Fix the problem if the arm is working when the car is moving
                rospy.logwarn(
                    "[Server] Car {} is moving, but arm is working, this should not happen!".format(msg.arm_id))
                return
            # The first call should be the arm finished picking, so the next move is to go the next target
            if self._cars[msg.arm_id - 1].arm_picking:
                rospy.loginfo(
                    "[Server] Car {} finished picking, sending goal {}...".format(msg.arm_id,
                                                                                  self._cars[msg.arm_id - 1]._next_target))
                self._cars[msg.arm_id - 1].arm_picking = False
                self._cars[msg.arm_id - 1].activate_car(goal_status=3)
                # The station where the car finished picking should mark itself as not occupied
                self._stations[(self._jobs[msg.job]["transfer"]["start"])].occupied_picking = False
                rospy.loginfo("[Server] Releasing station {}...".format(self._jobs[msg.job]["transfer"]["start"]))
            # The second call should be the arm finished dropping, so the next move is set the car to ready
            elif self._cars[msg.arm_id - 1].arm_dropping:
                rospy.loginfo("[Server] Car {} finished dropping, setting car to ready...".format(msg.arm_id))
                self._cars[msg.arm_id - 1].arm_dropping = False
                self._cars[msg.arm_id - 1].arm_working = False
                self._cars[msg.arm_id - 1].is_ready = True
                # Add car to available cars list
                self._available_cars_id.append(int(msg.arm_id))
                rospy.loginfo("[Server] Car {} is available list".format(msg.arm_id))
                # The car finished transporting the object, then add the station to working stations list
                if self._stations[self._jobs[msg.job]["transfer"]["end"]] != 0:
                    self._working_stations_idx.append(self._jobs[msg.job]["transfer"]["end"])
                if self._jobs[msg.job]["transfer"]["end"] == 0:
                    try:
                        self._working_jobs_name.remove(msg.job)
                    except Exception as e:
                        rospy.logerr_throttle(1, "Try to remove {} from working_jobs_name list but it doesn't exsist".format(msg.job))
                self._jobs[msg.job]["transfer"]["start"] = self._jobs[msg.job]["transfer"]["end"]
            self.print_all_available()
            return  # Don't need to check the rest

        rospy.logwarn_throttle(1, "[Server] Bad Arm ID: {}, skipping...".format(msg.arm_id))

    def get_dispatch_routes(self, job):
        # type: (str) -> tuple[int, int]
        """
        Get a job and return the start and end point indices
        First get the job's currently working station index
        Second get the job's next station index from available stations
        """
        start = self._jobs[job]["transfer"]["start"]
        # Check if the start station is not occupied
        if self._stations[start].occupied_picking:
            rospy.logwarn_throttle(1,
                                   "[Server] Start station {} is not available (others picking), returning -1".format(
                                       start))
            return -1, -1
        # if this is the last process of the job then go to distribution station
        if self._jobs[job]["process"] == 0:
            end = 0 
        else:
            try: 
                end = self._available_stations_idx.pop(0)
            except IndexError:
                rospy.logwarn_throttle(1, "[Server] No more available stations, returning -1")
                return -1, -1
            
        return start, end

    def start_stations(self):
        # type: () -> None
        """
        Check all stations in the working_stations list, if a station is not working, start it
        """
        if len(self._working_stations_idx) > 0:
            for station_index in self._working_stations_idx:
                if not self._stations[station_index].is_working:
                    rospy.loginfo("[Server] Starting station " + str(station_index + self._n_cars + 1))
                    self._stations[station_index].is_working = True
                    self._stations[station_index].start_arm()
            

    def has_available_jobs(self):
        # type: () -> bool
        if len(self._available_jobs_name) > 0:
            return True
        else:
            rospy.logwarn_throttle(1, "[Server] No more available jobs")
            return False

    def has_available_cars(self):
        # type: () -> bool
        if len(self._available_cars_id) > 0:
            return True
        else:
            rospy.logwarn_throttle(1, "[Server] No more available cars")
            return False

    def has_available_stations(self):
        # type: () -> bool
        if len(self._available_stations_idx) > 0:
            return True
        else:
            rospy.logwarn_throttle(1, "[Server] No more available stations")
            return False

    def all_job_done(self):
        # type: () -> bool
        for job in self._jobs.items():
            # If a job is not finished
            if job[1]["process"] >= 0:
                return False
            # If there are working job
            if len(self._working_jobs_name) > 0:
                return False
            # If a job is finished but not send back to distribution
            if job[1]["transfer"]["start"] != 0:
                return False
        return True

        # return len(self._available_jobs) == 0 and len(self._working_jobs) == 0

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
        if not self.has_available_jobs():
            rospy.logwarn_throttle(1, "[Server] Currently there is no job available when trying to dispatch")
            return 
        
        try:
            job = self._available_jobs_name[0]
        except IndexError as e:
            rospy.logerr_throttle(1, "[Server] Index out of range when trying to get a job, this is probably because there are no available jobs")
            return
        
        if not self.has_available_cars():
            rospy.logwarn_throttle(1, "[Server] Currently there is no car available when trying to dispatch")
            return
        
        try:
            car_id = self._available_cars_id[0]
        except IndexError as e:
            rospy.logerr_throttle(1, "[Server] Index out of range when trying to get a car, this is probably because there are no available cars")
            return
            
        route = self.get_dispatch_routes(job)
        if route == (-1, -1):
            rospy.logwarn_throttle(1, "[Server] Bad route, skipping...")
            return
        self._available_jobs_name.pop(0)
        self._available_cars_id.pop(0)
        rospy.loginfo("Get route for car " + str(car_id) + ": " + str(route))
        # Check car_index and point_index
        if car_id <= 0 or car_id > len(self._cars):
            rospy.logwarn_throttle(1, "Invalid car index")
            return
        if route[0] < 0 or route[0] >= len(self._stations):
            rospy.logwarn_throttle(1, "Invalid point index for car {}, skipping...".format(car_id))
            return
        
        # dispatch car to point and set mission object and set target indices
        try:
            car = self._cars[car_id - 1]
        except IndexError as e:
            rospy.logerr_throttle(1, "[Server] Index out of range when trying to get a car")
            return
        
        car.set_working_job_color(job, self._jobs[job]["target_color"])
        car.set_moving_targets(route[0], route[1])
        car.activate_car()
        # Lock the station that the car is picking up an object
        rospy.loginfo("[Server] Locking station {}...".format(route[0]))
        self._stations[route[0]].occupied_picking = True  # Set the station to occupied
        # Set the station's job that it will be doing. The station should be the route's end point
        self._stations[route[1]].set_job(job)
        self._stations[route[1]].set_working_color(self._jobs[job]["target_color"])
        # Add the car to working_car list
        self._working_cars_id.append(car_id)
        # Add the job to working_job list
        self._working_jobs_name.append(job)

        self._jobs[job]["process"] -= 1  # Decrease the job's process by 1
        self._jobs[job]["transfer"]["start"] = route[0]  # Set the job's starting station to the current station
        self._jobs[job]["transfer"]["end"] = route[1]  # Set the job's ending station to the next station

    def run(self):
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self._rate.sleep()
            if not self.all_job_done():
                if self.has_available_cars():
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
