import numpy as np
import tool_box
import time
import math
from arm_base import Arm_base
# from Arm_Lib import Arm_Device
class Arm_car(Arm_base):
    def __init__(self, ID, num_judgments,
                 depth=210, Arm_Location = (0, 0, 0), wucha = 1,
                 error_range = 12, wait_time = 12):
        super().__init__(ID, depth, Arm_Location, wucha, error_range, wait_time)
        self.num_judgments = num_judgments # 子类自己的参数

    def Arm_pick(self,color_information):
        # 师兄在外界调用的接口
        super().color_information(color_information) # 设置夹取的颜色信息
        # 发布消息给camera，准备识别
        super().sender(self.num_judgments)