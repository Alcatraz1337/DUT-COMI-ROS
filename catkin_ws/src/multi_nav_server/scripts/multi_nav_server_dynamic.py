#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class MultiNavServer:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self._rate = rospy.Rate(10)
        self.pub_markers = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_cancel_goal = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        self.sub_goal_result = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.goal_result_callback)
        self.sub_click = rospy.Subscriber('clicked_point', PointStamped, self.clicked_point_callback)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        self.goals = []
        self.goal_count = 0

        self.is_car_ready = True

    def shutdown(self):
        self.pub_cancel_goal.publish(GoalID())
        self.pub_cmd_vel.publish(Twist())
        # self.pub_markers.unregister()
        # self.pub_goal.unregister()
        # self.sub_goal_result.unregister()
        # self.sub_click.unregister()
        rospy.loginfo("Shutting down multi_nav_server")

    def clicked_point_callback(self, msg):
        # Handle clicked point
        # TODO: Not showing marker
        # print("Get new point" + str(msg))
        # marker = Marker()
        # marker.header.frame_id = 'map'
        # marker.header.stamp = rospy.Time.now()
        # marker.action = Marker.ADD
        # marker.scale.x = 0.5
        # marker.scale.y = 0.5
        # marker.scale.z = 0.5
        # marker.color.r = 255 / 255.0
        # marker.color.g = 33.0 / 255.0
        # marker.color.b = 113.0 / 255.0
        # marker.color.a = 1.0
        # marker.pose.position.x = msg.point.x
        # marker.pose.position.y = msg.point.y
        # marker.pose.position.z = msg.point.z
        # # Publish marker
        # self.pub_markers.publish(marker)
        
        # Add a goal to goal list
        goal = PoseStamped()
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 1
        print("Get new goal:\n" + str(goal))
        self.goals.append(goal)

        if self.is_car_ready:
            self.publish_target_goal_point()

    def goal_result_callback(self, msg):
        # Handle goal result
        print(msg.status.text)
        if msg.status.status == 3:
            # TODO: Handle goal reached
            self.is_car_ready = True
        if self.is_car_ready:
            self.publish_target_goal_point()
        

    def publish_target_goal_point(self):
        if len(self.goals) != 0:
            target = self.goals.pop(0)
            target.header.frame_id = 'map'
            target.header.stamp = rospy.Time.now()
            print("Publishing goal:\n" + str(target))
            # The location of the target should already be given
            self.pub_goal.publish(target)
            # Set the car to not ready
            self.is_car_ready = False
        else:
            print("No more goals")

if __name__ == '__main__':
    rospy.init_node('multi_nav_server')
    server = MultiNavServer()
    rospy.spin()
