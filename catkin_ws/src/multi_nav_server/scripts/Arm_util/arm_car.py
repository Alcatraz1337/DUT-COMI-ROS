#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import yaml
from arm_status_msgs.msg import ArmStatus
from arm_work_msgs.msg import ArmWork
from arm_base import Arm_base
from errorAnalyzer import ErrorAnalyzer
class Arm_car(Arm_base):
    def __init__(self, num_judgments=4,
                 depth=210, Arm_Location = (0, 0, 0), wucha = 1,
                 error_range = 12, wait_time = 12):
        super(Arm_car,self).__init__(depth, Arm_Location, wucha, error_range, wait_time)
        self._num_judgments = num_judgments # 子类自己的参数
        self._analyzer = ErrorAnalyzer(self._error_range, self._num_judgments)  # 生成误差分析对象 num_judgments,车是4，四角固定是15
        self._work = ''
        self._job = '' # Ros传过来的消息，告诉我们这是什么任务。
        self._example_flag = False

        self._msg = ArmStatus()
        self._msg.status = False # 机械臂的状态初始化为False
        # 消息的注册
        self.pub_arm_status = rospy.Publisher('arm_status', ArmStatus, queue_size=1)
        # 消息的订阅
        self.sub_arm_work = rospy.Subscriber('arm_work', ArmWork, self.call_back)
        print("car is complete")

    def call_back(self, msg):
        if self._ID == msg.arm_id:
            rospy.loginfo("arm_car: call back has been called")
            if not self._example_flag:
                # 获取消息的内容
                self._work = msg.work
                self._job = msg.job
                self.set_color_information(msg.color) # 设置夹取的颜色信息

                # 执行相应的命令，pick / drop任意一个干完就发一次消息
                if self._work == 'pick':
                    self.Arm_pick()
                    self.sender()  # ros返回消息
                    self.reset()  # 重置信息
                elif self._work == 'drop':
                    self.Arm_drop()
                    self.sender()  # ros返回消息
                    self.reset()  # 重置信息
                rospy.loginfo("arm_car: flag has been changed True")
                self._example_flag = True

    # 程序执行完毕后，一些必要的信息必须重置为开始状态
    def reset(self):
        self._color_information = None
        self._flag = False
        self._flag_error = False
        self._angle = None
        self._armStatic_status = False
        self._work = ''
        self._job = ''
        self._color_information = None

    # 机械臂夹取完毕后，返回消息，
    def sender(self):
        self._msg.arm_id = self._ID
        self._arm_status = True
        self._job = self._job

        # 发送消息
        self.pub_arm_status.publish(self._msg)
        rospy.loginfo("arm_car: pub_arm_status has been pubulished")

    def Arm_pick(self):
        # 开始识别识别，返回角度
        self._angle = self.recognize_angle(self._analyzer)
        # 夹取
        self.execute_car_pick()

    def Arm_drop(self):
        # 放置
        self.execute_car_place()

    # 小车机械臂的夹取函数
    def execute_car_pick(self):
        # 旋转
        if abs(self._angle) < 90:
            self.control_Arm(1, 90 - self._angle, False, self._wucha)
        elif abs(self._angle) > 90:
            self.control_Arm(1, 90 - abs(self._angle), True, self._wucha)
        rospy.sleep(.5)
        # 准备夹取
        # 1、先伸直
        angle_id_1 = self._Arm.Arm_serial_servo_read(1)  # 读取第一舵机的角度
        p_move_1 = [angle_id_1, 90, 0, 90, 90]
        # 2、机械臂伸直
        self.arm_move(p_move_1, 1500)
        rospy.sleep(2)
        # 机械臂2和4移动
        self._Arm.Arm_serial_servo_write(4, 80, 1500)
        rospy.sleep(2)
        self._Arm.Arm_serial_servo_write(2, 53, 1500)
        rospy.sleep(2)
        # 夹取
        self.arm_clamp_block(1)  # 夹取
        rospy.sleep(1)
        # 回归到等待位置
        self.arm_move(self._p_mould, 1000)  # 回到指定位置

    # 小车机械臂的放置函数，参数是举例的，具体的参数需要根据实际情况进行修改
    def execute_car_place(self):
        if self.get_color_information() == 'Red':
            angle_id_1 = 85
            angle_id_2 = 53
            angle_id_4 = 80
            p_move_1 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_1, 1500)  # 机械臂伸到放置红色方块的地方
            rospy.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            rospy.sleep(1)
            self.arm_move(self._p_mould, 1500)
        if self.get_color_information() == 'Yellow':
            angle_id_1 = 100
            angle_id_2 = 53
            angle_id_4 = 80
            p_move_2 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_2, 1500)  # 机械臂伸到放置红色方块的地方
            rospy.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            rospy.sleep(1)
            self.arm_move(self._p_mould, 1500)
        if self.get_color_information() == 'Blue':
            angle_id_1 = 75
            angle_id_2 = 53
            angle_id_4 = 80
            p_move_3 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_3, 1500)  # 机械臂伸到放置红色方块的地方
            rospy.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            rospy.sleep(1)
            self.arm_move(self._p_mould, 1500)