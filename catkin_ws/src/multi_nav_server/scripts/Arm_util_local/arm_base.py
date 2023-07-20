#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import tool_box
# from arm_work_msgs.msg import ArmWork
from errorAnalyzer import ErrorAnalyzer
from Arm_Lib import Arm_Device

class Arm_base:
    def __init__(self,
                 depth=210, Arm_Location=(0, 0, 0), wucha=1,
                 error_range=12, wait_time=12):
        self._ID = -1  # 每个机械臂的唯一标识
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
        self._p_mould = [90, 130, 0, 0, 90]  # 机械臂的等待位置


    def set_color_information(self, color_information): # 通过ros消息得到颜色信息
        self.color_information = color_information

    def get_color_information(self):
        return self.color_information

    def set_id(self, ID):
        self._ID = ID
    def get_id(self):
        return self._ID

    def Arm_pick(self):
        pass
        # 在子类里具体细化

    def Arm_drop(self):
        pass
        # 在子类里具体细化

    def execute_rotate(self, num_judgments):
        # ros给Camera发送消息
        self._angle = self.recognize_angle(num_judgments)  # 返回计算所得的angle
        # 旋转
        if abs(self._angle) < 90:
            self.control_Arm(id, 90 - self._angle, False, self._wucha)
        elif abs(self._angle) > 90:
            self.control_Arm(id, 90 - abs(self._angle), True, self._wucha)

    def recognize_angle(self,num_judgments):
        wait_time = 12
        Arm_Location = (0, 0, 0)
        coordinates = []

        ser = tool_box.open_serial()  # 打开串口,返回串口对象
        analyzer = ErrorAnalyzer(self._error_range, num_judgments)  # 生成误差分析对象 对于num_judgments,车是4，四角固定是15
        # 启动串口
        rospy.loginfo("串口正在启动, 请等待{time}秒……".format(time=wait_time))
        # 倒计时显示
        for i in range(wait_time, 0, -1):
            rospy.loginfo("\r", "倒计时{}秒！".format(i), end="", flush=True)
            time.sleep(1)
        # 循环
        while self.flag == False:
            if (ser.in_waiting > 0):
                result, flag = tool_box.setial_communicatiuon_mutiple(ser)
                if result is not None:
                    flag, coordinates = tool_box.check_coloridx_imformation(result, self.get_color_information())
                    # 对coordinates的第一位，也就是x值进行误差分析
                    if self.flag_error == False:
                        rospy.loginfo("正在对获取的数据进行误差分析……")
                        valid_data, self.flag_error = analyzer.analyze(coordinates[0])
                        if self.flag_error:
                            rospy.loginfo("误差分析已完毕，坐标已获取，信息如下：")
                            rospy.loginfo(coordinates)
                            time.sleep(1)
                        else:
                            self.flag = False  # 如果误差分析不合格，则继续获取数据
                            time.sleep(1)
                else:
                    rospy.loginfo("坐标数据不准确：")
                    continue
            else:
                rospy.loginfo("串口中无数据……")
                time.sleep(.5)
        # 获取到目标的图像坐标，接下来转换成世界坐标
        pixel_coords = coordinates
        rospy.loginfo("图像坐标：", pixel_coords)
        # 图像是1280*960，但是我们的相机LCD显示的是320*240,所以要*4，将分辨率调回来，坐标才会准确
        image_points = tuple(x * 4 for x in pixel_coords)
        # 计算世界坐标
        color_x_point, color_y_point = tool_box.image_to_world_v2(self._depth, image_points)
        world_point = (color_x_point, color_y_point, self._depth)
        rospy.loginfo(world_point)
        # 计算旋转角度
        angle = tool_box.azimuthangle(Arm_Location, world_point)
        rospy.loginfo("angle:", angle)
        return angle

    # 定义夹积木块函数，enable=1：夹住，=0：松开
    def arm_clamp_block(self, enable):
        if enable == 0:
            self._Arm.Arm_serial_servo_write(6, 60, 400)
        else:
            # Arm.Arm_serial_servo_write(6, 135, 400)
            self._Arm.Arm_serial_servo_write(6, 112, 400)  # 112，夹子刚好可以夹紧物坄1�7
        time.sleep(.5)

    # 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
    def arm_move(self, p, s_time=500):
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

