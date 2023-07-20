#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
from arm_base import Arm_base
class Arm_static(Arm_base):
    def __init__(self, ID, num_judgments=15,
                 depth=210, Arm_Location = (0, 0, 0), wucha = 1,
                 error_range = 12, wait_time = 12):
        super().__init__(ID, depth, Arm_Location, wucha, error_range, wait_time)
        self._num_judgments = num_judgments # 子类自己的参数

    def Arm_execute(self,color_information):
        # 师兄在外界调用的接口
        super().set_color_information(color_information) # 设置夹取的颜色信息
        # 发布消息给camera，准备识别，并旋转到指定角度
        super().sender(self._num_judgments)
        # 夹取
        self.execute_static_pick()
        # 放置
        self.execute_static_place()
        # 重置一些必要的信息
        self.reset()

    def execute_static_pick(self):# 四角机械臂的夹取函数,最后实地测试的时候，这个函数的参数需要修改
        # 机械臂夹取
        # 1、先伸直
        angle_id_1 = self._Arm.Arm_serial_servo_read(1)  # 读取第一舵机的角度
        p_move_1 = [angle_id_1, 90, 0, 90, 90]
        self.arm_move(p_move_1, 1500)  # 机械臂伸直
        time.sleep(2)
        self._Arm.Arm_serial_servo_write(4, 80, 1500)
        time.sleep(2)
        self._Arm.Arm_serial_servo_write(2, 53, 1500)
        time.sleep(2)
        self.arm_clamp_block(1)  # 夹取
        time.sleep(1)
        p_mould = [90, 130, 0, 0, 90]
        self.arm_move(p_mould, 1000)  # 回到指定位置
    def execute_static_place(self):
        if self._color_information == 'Red':  # 是红色，放置在左上角的方格中
            angle_id_1 = 161
            angle_id_2 = 53
            angle_id_4 = 80
            p_move_1 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_1, 1500)  # 机械臂伸到放置红色方块的地方
            time.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            time.sleep(1)
            self.arm_move(self._p_mould, 1500)

        if self._color_information == "Yellow":
            angle_id_1 = 156
            angle_id_2 = 60
            angle_id_4 = 55
            p_move_1 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_1, 1500)  # 机械臂伸到放置红色方块的地方
            time.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            time.sleep(1)
            self.arm_move(self._p_mould, 1500)

        if self._color_information == "Blue":
            angle_id_1 = 172
            angle_id_2 = 45
            angle_id_4 = 90
            p_move_1 = [angle_id_1, angle_id_2, 0, angle_id_4, 90]  # 应该放置红色方块的地方
            self.arm_move(p_move_1, 1500)  # 机械臂伸到放置红色方块的地方
            time.sleep(2)
            self.arm_clamp_block(0)  # 松开夹子
            time.sleep(1)
            self.arm_move(self._p_mould, 1500)