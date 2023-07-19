import tool_box
import time
#!/usr/bin/env python
# -*- coding: utf-8 -*-
from arm_base import Arm_base
class Arm_car(Arm_base):
    def __init__(self, ID, num_judgments,
                 depth=210, Arm_Location = (0, 0, 0), wucha = 1,
                 error_range = 12, wait_time = 12):
        super().__init__(ID, depth, Arm_Location, wucha, error_range, wait_time)
        self.num_judgments = num_judgments # 子类自己的参数

    def Arm_pick(self,color_information):
        # 师兄在外界调用的接口
        super().set_color_information(color_information) # 设置夹取的颜色信息
        # 发布消息给camera，准备识别
        super().sender(self.num_judgments)

    # 小车机械臂的夹取函数
    def execute_car_pick(self):
        # 1、先伸直
        angle_id_1 = self.Arm.Arm_serial_servo_read(1)  # 读取第一舵机的角度
        p_move_1 = [angle_id_1, 90, 0, 90, 90]
        # 2、机械臂伸直
        self.arm_move(p_move_1, 1500)
        time.sleep(2)
        # 机械臂2和4移动
        self.Arm.Arm_serial_servo_write(4, 80, 1500)
        time.sleep(2)
        self.Arm.Arm_serial_servo_write(2, 53, 1500)
        time.sleep(2)
        # 夹取
        self.arm_clamp_block(1)  # 夹取
        time.sleep(1)
        # 回归到等待位置
        # p_mould = [90, 130, 0, 0, 90]
        self.arm_move(self.p_mould, 1000)  # 回到指定位置
    # 小车机械臂的放置函数
    def execute_car_place(self):
        pass
