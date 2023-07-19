#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import math
from arm_base import Arm_base
class Arm_static(Arm_base):
    def __init__(self, ID, num_judgments,
                 depth=210, Arm_Location = (0, 0, 0), wucha = 1,
                 error_range = 12, wait_time = 12):
        super().__init__(ID, depth, Arm_Location, wucha, error_range, wait_time)
        self.num_judgments = num_judgments # 子类自己的参数

    def Arm_pick(self,color_information):
        # 师兄在外界调用的接口
        super().set_color_information(color_information) # 设置夹取的颜色信息
        # 执行夹取命令
        super().execute_pick(self.num_judgments)

    # 四角机械臂的夹取函数
    def execute_static_pick(self):
        # 机械臂夹取
        # 1、先伸直
        angle_id_1 = self.Arm.Arm_serial_servo_read(1)  # 读取第一舵机的角度
        p_move_1 = [angle_id_1, 90, 0, 90, 90]
        self.arm_move(p_move_1, 1500)  # 机械臂伸直
        time.sleep(2)
        self.Arm.Arm_serial_servo_write(4, 80, 1500)
        time.sleep(2)
        self.Arm.Arm_serial_servo_write(2, 53, 1500)
        time.sleep(2)
        self.arm_clamp_block(1)  # 夹取
        time.sleep(1)
        p_mould = [90, 130, 0, 0, 90]
        self.arm_move(p_mould, 1000)  # 回到指定位置