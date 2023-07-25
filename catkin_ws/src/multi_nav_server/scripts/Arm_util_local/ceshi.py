#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys

sys.path.append('/home/jetson/CodeSpace/catkin_ws/devel/lib/python2.7/dist-packages/')
import rospy
from arm_status_msgs.msg import ArmStatus
from arm_work_msgs.msg import ArmWork
from arm_car import Arm_car


# 模拟服务器发消息给小车对象
def sender(msg, pub_arm_work_wai):
    msg.arm_id = 0
    msg.work = 'pick'
    msg.job = 'job1'
    msg.color = 'Yellow'

    # 发布消息
    pub_arm_work_wai.publish(msg)
    # rospy.loginfo("message is published")
    rospy.loginfo_once("message is published")


def call_back(msg):
    print("the return message is, id:{}, arm_atatus:{}, arm_job:{}".format(msg.id, msg.status, msg.job))


# 初始化ROS节点
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10)
# 消息的订阅
sub_arm_status = rospy.Subscriber('arm_status', ArmStatus, call_back)
# 消息的注册
pub_arm_work = rospy.Publisher('arm_work', ArmWork, queue_size=1)

# 执行程序,等待消息的传入
arm_car = Arm_car()
flag = False
while not rospy.is_shutdown():
    msg = ArmWork()
    sender(msg, pub_arm_work)
    rate.sleep()
