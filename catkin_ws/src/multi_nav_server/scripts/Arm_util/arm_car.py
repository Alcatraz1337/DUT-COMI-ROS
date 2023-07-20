#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
from Camera_msgs.msg import Camera_Angle, Camera_Target,Camera_Status
from arm_base import Arm_base
import rospy
class Arm_car(Arm_base):
    def __init__(self, ID, num_judgments=4,
                 depth=210, Arm_Location = (0, 0, 0), wucha = 1,
                 error_range = 12, wait_time = 12):
        super().__init__(ID, depth, Arm_Location, wucha, error_range, wait_time)
        self._num_judgments = num_judgments # 子类自己的参数

    def Arm_execute(self,color_information):
        # 师兄在外界调用的接口
        super().set_color_information(color_information) # 设置夹取的颜色信息
        # 发布消息给camera，准备识别
        super().sender(self._num_judgments)
        # 夹取
        self.execute_car_pick()
        # 放置
        self.execute_car_place()
        # 通过ros发布消息给小车，告诉小车可以走了
        self.sender_car()
        # 重置一些信息
        self.reset()
    def sender_car(self):
        msg = Camera_Status()
        msg.ID = self._ID
        msg.status = True
        self.pub_camera_status.publish(msg)
        # 重新置为初始状态
        msg.status = False
        rospy.loginfo("sender_car is completed……")
    # 小车机械臂的夹取函数
    def execute_car_pick(self):
        # 1、先伸直
        angle_id_1 = self._Arm.Arm_serial_servo_read(1)  # 读取第一舵机的角度
        p_move_1 = [angle_id_1, 90, 0, 90, 90]
        # 2、机械臂伸直
        self.arm_move(p_move_1, 1500)
        time.sleep(2)
        # 机械臂2和4移动
        self._Arm.Arm_serial_servo_write(4, 80, 1500)
        time.sleep(2)
        self._Arm.Arm_serial_servo_write(2, 53, 1500)
        time.sleep(2)
        # 夹取
        self.arm_clamp_block(1)  # 夹取
        time.sleep(1)
        # 回归到等待位置
        # p_mould = [90, 130, 0, 0, 90]
        self.arm_move(self._p_mould, 1000)  # 回到指定位置

    # 小车机械臂的放置函数
    def execute_car_place(self):
        if self._color_information == 'Red':
            angle_id_1 = 85
            angle_id_2 = 53
            angle_id_4 = 80
            p_move_1 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_1, 1500)  # 机械臂伸到放置红色方块的地方
            time.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            time.sleep(1)
            self.arm_move(self._p_mould, 1500)
        if self._color_information == 'Yellow':
            angle_id_1 = 100
            angle_id_2 = 53
            angle_id_4 = 80
            p_move_2 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_2, 1500)  # 机械臂伸到放置红色方块的地方
            time.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            time.sleep(1)
            self.arm_move(self._p_mould, 1500)
        if self._color_information == 'Blue':
            angle_id_1 = 75
            angle_id_2 = 53
            angle_id_4 = 80
            p_move_3 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_3, 1500)  # 机械臂伸到放置红色方块的地方
            time.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            time.sleep(1)
            self.arm_move(self._p_mould, 1500)