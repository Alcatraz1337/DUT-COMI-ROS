#!/usr/bin/env python

import time
import tool_box
import rospy
from errorAnalyzer import ErrorAnalyzer
from Camera_msgs.msg import Camera_Target, Camera_Angle

pub_camera_angle = rospy.Publisher("Camera_Angle",Camera_Angle,queue_size=1)
target = ""
flag = False
flag_error = False
error_range = 0
num_judgments = 0
depth = 0
def call_back(msg):
    global flag = msg.flag
    global flag_error = msg.flag_error
    global depth = msg.depth
    global error_range = msg.error_range
    global num_judgments = msg.num_judgements

    # 获取到角度,更改target信息
    global target = msg.color_information
# 搜索对象
sub_camera_target = rospy.Subscriber('Camera_Target', Camera_Target, call_back)

def recognize():
    wait_time = 12
    Arm_Location = (0, 0, 0)
    coordinates = []

    ser = tool_box.open_serial()  # 打开串口,返回串口对象
    analyzer = ErrorAnalyzer(error_range, num_judgments)  # 生成误差分析对象 对于num_judgments,车是4，四角固定是15
    # 启动串口
    rospy.loginfo("串口正在启动, 请等待{time}秒……".format(time=wait_time))
    # 倒计时显示
    for i in range(wait_time, 0, -1):
        rospy.loginfo("\r", "倒计时{}秒！".format(i), end="", flush=True)
        time.sleep(1)
    # 循环
    while flag == False:
        if (ser.in_waiting > 0):
            result, flag = tool_box.setial_communicatiuon_mutiple(ser)
            if result is not None:
                flag, coordinates = tool_box.check_coloridx_imformation(result, msg.color_information)
                # 对coordinates的第一位，也就是x值进行误差分析
                if flag_error == False:
                    rospy.loginfo("正在对获取的数据进行误差分析……")
                    valid_data, flag_error = analyzer.analyze(coordinates[0])
                    if flag_error:
                        rospy.loginfo("误差分析已完毕，坐标已获取，信息如下：")
                        rospy.loginfo(coordinates)
                        time.sleep(1)
                    else:
                        flag = False  # 如果误差分析不合格，则继续获取数据
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
    color_x_point, color_y_point = tool_box.image_to_world_v2(depth, image_points)
    world_point = (color_x_point, color_y_point, depth)
    rospy.loginfo(world_point)
    # 计算旋转角度
    angle = tool_box.azimuthangle(Arm_Location, world_point)
    rospy.loginfo("angle:", angle)
    return angle


def sender(angle):
    msg = Camera_Angle()
    msg.angle = angle

    # ros发布，返回信息
    pub_camera_angle.publish("Camera_Angle", Camera_Angle, queue_size=1)

def shutdown():
    pub_camera_angle.unregister()
    sub_camera_target.unregister()

while not rospy.shutdown(shutdown):
    # ros发布，返回信息
    if target != "":
        # TODO: 识别(考虑识别不到的情况)
        target_angle = recognize()
        sender(target_angle)
        target = ""

rospy.loginfo("camera shutting down...")