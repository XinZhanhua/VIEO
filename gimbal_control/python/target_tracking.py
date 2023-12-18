#!/home/fishyu/anaconda3/envs/ros_env/bin/python3.11

import rospy
from geometry_msgs.msg import TransformStamped
import serial
import tf

# 全局变量
global_pose = None

def tf_callback(data):
    global global_pose
    global_pose = data.transform

rospy.init_node('target_tracking')
rospy.Subscriber('/tf', TransformStamped, tf_callback)

rate = rospy.Rate(100)  # 设置循环频率为100Hz

# 打开串口
ser = serial.Serial('/dev/ttyUSB0', 921600)

class PIDController:
    def __init__(self):
        # 初始化PID控制器参数
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0

    def calculate(self, current_value, target_value):
        # 计算PID控制器输出
        error = target_value - current_value
        self.error_sum += error
        error_diff = error - self.last_error
        output = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_diff
        self.last_error = error
        return output

# 建立PID控制器
pid_controller = PIDController()

while not rospy.is_shutdown():
    if global_pose is not None:
        rotation = global_pose.rotation
        euler = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        yaw = euler[2]  # 获取yaw轴角度
        
        # 使用yaw的角度值作为反馈，控制电机转动
        motor_control = pid_controller.calculate(yaw, 0)
        speed = motor_control * 2 * 3.1415926536
        control_high = int(speed * 100) >> 8
        control_low = int(speed * 100) & 0xFF
        data = [0xAA, 0xAF, 0x05, 0x13, 0x02, control_high, control_low]
        checksum = sum(data) & 0xFF
        data.append(checksum)
        data_bytes = bytes(data)
        ser.write(data_bytes)
    #rospy.loginfo("yes")
    rate.sleep()

