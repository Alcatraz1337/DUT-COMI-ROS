import time
import rospy
from Camera_msgs.msg import Camera_Angle, Camera_Target
from Arm_Lib import Arm_Device

class Arm_base:
    def __init__(self, ID,
                 depth=210, Arm_Location=(0, 0, 0), wucha=1,
                 error_range=12, wait_time=12):
        self.ID = ID  # 每个机械臂的唯一标识
        self.color_information = None  # 服务器传来的颜色信息
        self.depth = depth  # 镜头距离物块的深度
        self.Arm_Location = Arm_Location  #
        self.wucha = wucha  # 旋转角度误差
        self.wait_time = wait_time  # 等待索尼相机打开后稳定下来的时间
        self.flag = False  # 循环标识位
        self.flag_error = False  # 误差标识位
        self.error_range = error_range  # 识别框的误差允许范围
        self.Arm = Arm_Device()  # 初始化的时就应该生成机械臂对象
        self.angle = None  # 初始化是角度为空
        self.p_mould = [90, 130, 0, 0, 90] # 机械臂的等待位置
        # 初始化时注册消息的发布
        self.pub_camera_target = rospy.Publisher("Camera_Target", Camera_Target, queue_size=1)
        self.pub_camera_angle = rospy.Publisher("Camera_Angle", Camera_Angle, queue_size=1)
        # 消息的订阅
        self.sub_camera_angle = rospy.Subscriber('Camera_Angle', Camera_Angle, self.execute_pick)
    def set_color_information(self, color_information):
        self.color_information = color_information

    def get_color_information(self):
        return self.color_information

    def set_id(self, ID):
        self.ID = ID
    def get_ID(self):
        return self.ID

    def reset(self):  # 程序执行完毕后，一些必要的信息必须重置为开始状态
        self.color_information = None
        self.flag = False
        self.flag_error = False

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
        msg.ID = self.ID
        msg.flag = self.flag
        msg.flag_error = self.flag_error
        msg.color_information = self.get_color_information()
        msg.depth = self.depth
        msg.error_range = self.error_range
        msg.num_judgments = num_judgments

        # 发布消息
        self.pub_camera_target.publish(msg)

    def execute_pick(self, msg):
        # ros给Camera发送消息
        self.angle = msg.angle  # 返回计算所得的angle

        if abs(self.angle) < 90:
            self.control_Arm(id, 90 - self.angle, False, self.wucha)
        elif abs(self.angle) > 90:
            self.control_Arm(id, 90 - abs(self.angle), True, self.wucha)
        #


    # 定义夹积木块函数，enable=1：夹住，=0：松开
    def arm_clamp_block(self, enable):
        if enable == 0:
            self.Arm.Arm_serial_servo_write(6, 60, 400)
        else:
            # Arm.Arm_serial_servo_write(6, 135, 400)
            self.Arm.Arm_serial_servo_write(6, 112, 400)  # 112，夹子刚好可以夹紧物坄1�7
        time.sleep(.5)

    # 定义移动机械臂函敄1�7,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
    def arm_move(self, p, s_time=500):
        for i in range(5):
            id = i + 1
            if id == 5:
                time.sleep(.1)
                self.Arm.Arm_serial_servo_write(id, p[i], int(s_time * 1.2))
            else:
                self.Arm.Arm_serial_servo_write(id, p[i], s_time)
            time.sleep(.01)
        time.sleep(s_time / 1000)

    def control_Arm(self, id, angle, flag, wucha):
        if flag == False:
            angle = self.Arm.Arm_serial_servo_read(id) - (abs(angle) + wucha)
            time.sleep(1)
            self.Arm.Arm_serial_servo_write(id, angle, 1500)
            time.sleep(1)
        else:
            angle = self.Arm.Arm_serial_servo_read(id) + (abs(angle) + wucha)
            time.sleep(1)
            self.Arm.Arm_serial_servo_write(id, angle, 1500)
            time.sleep(1)

    def arm_centrality(self):
        # 定义不同位置的变量参数
        p_mould = [90, 130, 0, 0, 90]
        # 让机械臂移动到一个准备抓取的位置
        self.arm_clamp_block(0)
        self.arm_move(p_mould, 1000)
        time.sleep(1)
