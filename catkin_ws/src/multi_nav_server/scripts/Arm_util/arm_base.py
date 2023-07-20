#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from Camera_msgs.msg import Camera_Angle, Camera_Target,Camera_Status
from Arm_Lib import Arm_Device

class Arm_base:
    def __init__(self, ID,
                 depth=210, Arm_Location=(0, 0, 0), wucha=1,
                 error_range=12, wait_time=12):
        self._ID = ID  # 每个机械臂的唯一标识
        self._color_information = None  # 服务器传来的颜色信息
        self._depth = depth  # 镜头距离物块的深度
        self._Arm_Location = Arm_Location  # 坐标系的原点在机械臂的位置
        self._wucha = wucha  # 旋转角度误差
        self._wait_time = wait_time  # 等待索尼相机打开后稳定下来的时间
        self._flag = False  # 循环标识位
        self._flag_error = False  # 误差标识位
        self._error_range = error_range  # 识别框的误差允许范围
        self._Arm = Arm_Device()  # 初始化的时就应该生成机械臂对象
        self._angle = None  # 初始化是角度为空
        self._p_mould = [90, 130, 0, 0, 90] # 机械臂的等待位置
        # 初始化市对Camera_Status消息的状态码进行初始化
        self._camera_status = Camera_Status()
        self._camera_status.status = False
        # 初始化时注册消息的发布
        self.pub_camera_target = rospy.Publisher("Camera_Target", Camera_Target, queue_size=1)
        self.pub_camera_angle = rospy.Publisher("Camera_Angle", Camera_Angle, queue_size=1)
        self.pub_camera_status = rospy.Publisher("Camera_Status", Camera_Status, queue_size=1)
        # 消息的订阅
        self.sub_camera_angle = rospy.Subscriber('Camera_Angle', Camera_Angle, self.execute_rotate)
    def set_color_information(self, color_information):
        self._color_information = color_information

    def get_color_information(self):
        return self._color_information

    def set_id(self, ID):
        self._ID = ID
    def get_ID(self):
        return self._ID

    def reset(self):  # 程序执行完毕后，一些必要的信息必须重置为开始状态
        self._color_information = None
        self._flag = False
        self._flag_error = False

    def Arm_pick(self, color_information):
        pass
        # 在子类里具体细化

    def Arm_drop(self):
        pass
        # 在子类里具体细化

    def sender(self, num_judgments):
        # 创建消息对象
        msg = Camera_Target()
        # 信息打包
        msg.ID = self._ID
        msg.flag = self._flag
        msg.flag_error = self._flag_error
        msg.color_information = self.get_color_information()
        msg.depth = self._depth
        msg.error_range = self._error_range
        msg.num_judgments = num_judgments

        # 发布消息
        self.pub_camera_target.publish(msg)

    def execute_rotate(self, msg): # 回调函数
        # ros给Camera发送消息
        self._angle = msg.angle  # 返回计算所得的angle
        # 旋转
        if abs(self._angle) < 90:
            self.control_Arm(id, 90 - self._angle, False, self._wucha)
        elif abs(self._angle) > 90:
            self.control_Arm(id, 90 - abs(self._angle), True, self._wucha)


    def arm_clamp_block(self, enable): # 定义夹积木块函数，enable=1：夹住，=0：松开
        if enable == 0:
            self._Arm.Arm_serial_servo_write(6, 60, 400)
        else:
            # Arm.Arm_serial_servo_write(6, 135, 400)
            self._Arm.Arm_serial_servo_write(6, 112, 400)  # 112，夹子刚好可以夹紧物块
        time.sleep(.5)

    def arm_move(self, p, s_time=500):# 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
        for i in range(5):
            id = i + 1
            if id == 5:
                time.sleep(.1)
                self._Arm.Arm_serial_servo_write(id, p[i], int(s_time * 1.2))
            else:
                self._Arm.Arm_serial_servo_write(id, p[i], s_time)
            time.sleep(.01)
        time.sleep(s_time / 1000)

    def control_Arm(self, id, angle, flag, wucha):
        if flag == False:
            angle = self._Arm.Arm_serial_servo_read(id) - (abs(angle) + wucha)
            time.sleep(1)
            self._Arm.Arm_serial_servo_write(id, angle, 1500)
            time.sleep(1)
        else:
            angle = self._Arm.Arm_serial_servo_read(id) + (abs(angle) + wucha)
            time.sleep(1)
            self._Arm.Arm_serial_servo_write(id, angle, 1500)
            time.sleep(1)

    def arm_centrality(self):
        # 定义不同位置的变量参数
        p_mould = [90, 130, 0, 0, 90]
        # 让机械臂移动到一个准备抓取的位置
        self.arm_clamp_block(0)
        self.arm_move(p_mould, 1000)
        time.sleep(1)

    def execute_pick(self, msg):
        # ros给Camera发送消息
        self._angle = msg.angle  # 返回计算所得的angle

        if abs(self._angle) < 90:
            self.control_Arm(id, 90 - self._angle, False, self._wucha)
        elif abs(self._angle) > 90:
            self.control_Arm(id, 90 - abs(self._angle), True, self._wucha)