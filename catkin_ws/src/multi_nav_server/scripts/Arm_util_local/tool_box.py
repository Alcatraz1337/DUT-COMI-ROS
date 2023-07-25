#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import serial.tools.list_ports
from decimal import Decimal, getcontext
import numpy as np
import math


def open_serial():
    sername = None
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备。")
    else:
        print("可用的串口设备如下：")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])
            sername = list(comport)[0]
    # 配置串行通信参数
    # port = "COM9"  # 根据你的系统配置正确的串口设备 \Device\USBPDO-10
    # port = "/dev/ttyUSB1" # Nano path
    port = sername  # Nano path
    baud_rate = 115200  # 波特率设置，与 Spresense 开发板上的设置保持一致

    ser = serial.Serial(port, baud_rate)  # 打开COM13，将波特率配置为115200，其余参数使用默认值
    if ser.isOpen():  # 判断串口是否成功打开
        print("打开串口成功。")
        print(ser.name)  # 输出串口号
    else:
        print("打开串口失败。")
    return ser


def setial_communicatiuon_new(ser):
    coordinates = None
    # 接收从 Spresense 传来的指定数据
    # print("111")
    data = ser.readline().decode().strip()  # 解码并去除首尾的空白字符
    # print("222")
    if data[:4] == 'Data':
        # 解析坐标数据
        head, x, y, width, height = data.split(',')
        x = int(x)
        y = int(y)
        width = int(width)
        height = int(height)
        coordinates = (x, y, width, height)
        return coordinates, True
    else:
        return None, False


def setial_communicatiuon_mutiple(ser):
    coordinates = None
    prefix = color_red = red_x = red_y = width_red = height_red = color_yellow = yellow_x = yellow_y = width_y = height_y = color_blue = blue_x = blue_y = width_b = height_b = color_green = green_x = green_y = width_g = height_g = None
    # print("111")
    # 接收从 Spresense 传来的指定数据
    data = ser.readline().decode().strip()  # 解码并去除首尾的空白字符
    # print("222")
    if data[:4] == 'Data':
        # 解析坐标数据
        prefix, color_red, red_x, red_y, width_red, height_red, color_yellow, yellow_x, yellow_y, width_y, height_y, color_blue, blue_x, blue_y, width_b, height_b, color_green, green_x, green_y, width_g, height_g = data.split(
            ',')

        coordinates = [
        color_red, int(red_x), int(red_y), int(width_red), int(height_red), color_yellow, int(yellow_x), int(yellow_y),
        int(width_y), int(height_y), color_blue, int(blue_x), int(blue_y), int(width_b), int(height_b), color_green,
        int(green_x), int(green_y), int(width_g), int(height_g)]

        # print("Received data:", data)
        # print("coordinates  :",coordinates)

        return coordinates, True

    return coordinates, False


def check_coloridx_imformation(result, color_information):
    # 初始默认值
    x = y = -1
    width = height = -1
    flag = False
    # 红黄蓝绿
    # 如果是红色，且result[0]处的红色信息标识位不为空(-1代表空)，则从1,2,3,4三个位置取信息，下同
    if color_information == 'Red' and result[0] != '-1':
        x = result[1]
        y = result[2]
        width = result[3]
        height = result[4]
        flag = True
    if color_information == 'Yellow' and result[5] != '-1':
        x = result[6]
        y = result[7]
        width = result[8]
        height = result[9]
        flag = True
    if color_information == 'Blue' and result[10] != '-1':
        x = result[11]
        y = result[12]
        width = result[13]
        height = result[14]
        flag = True
    if color_information == 'Green' and result[15] != '-1':
        x = result[16]
        y = result[17]
        width = result[18]
        height = result[19]
        flag = True
    return flag, [x, y, width, height]


# 方法更新-v1
def image_to_world_v1(z_points, image_points):
    # 读取摄像机标定结果
    camera_matrix = np.array([[1007.2679860607, 0, 637.073198188448],
                              [0, 1008.59657130037, 487.69741803658],
                              [0, 0, 1]], dtype=np.float64)
    fx = image_points[0] + image_points[2] / 2
    fy = image_points[1] + image_points[3] / 2
    x_points = (z_points * (fx - camera_matrix[0][2])) / camera_matrix[0][0]
    y_points = (z_points * (fy - camera_matrix[1][2])) / camera_matrix[1][1]

    return x_points, y_points


# 方法更新-v2: COM13新镜头参数
def image_to_world_v2(z_points, image_points):
    # 读取摄像机标定结果
    camera_matrix = np.array([[996.481060885886, 0, 635.730545034981],
                              [0, 995.907916253828, 486.726116189548],
                              [0, 0, 1]], dtype=np.float64)
    fx = image_points[0] + image_points[2]
    fy = image_points[1] + image_points[3]
    # fx = image_points[0]
    # fy = image_points[1]
    x_points = (z_points * (fx - camera_matrix[0][2])) / camera_matrix[0][0]
    y_points = (z_points * (fy - camera_matrix[1][2])) / camera_matrix[1][1]

    return x_points, y_points


def calculate_rotation_angle(point1, point2):
    getcontext().prec = 10  # 设置 Decimal 类的精度为 10，用于处理更精确的浮点数计算。
    x1, z1 = point1[0], point1[2]
    x2, z2 = point2[0], point2[2]
    dx = Decimal(x2 - x1)
    dy = Decimal(z2 - z1)

    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    return angle_deg


def azimuthangle(point1, point2):
    x = math.atan2(point2[2] - point1[2], point2[0] - point1[0])
    x = x * 180 / math.pi  # 转换为角度
    if x < 0:
        x = 360 + x
    return x



