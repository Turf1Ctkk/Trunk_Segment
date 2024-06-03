#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PointStamped, Twist
import threading
from std_msgs.msg import String, Float32
import time
import serial
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class SystemState:
    NO_TARGET = "无目标"
    UNPROCESSED = "未处理"
    WAITING_FOR_ACTIONS = "等待动作完成"
    GRIPPING = "夹取中"
    PAINTING = "喷涂中"
    RELEASING = "释放中"
    PROCESSED = "已处理"

system_state = SystemState.PROCESSED

# 滑台参数
SLIDE_MAX_Y = 1000

GRIPPER_R = 0.8

vehicle_threads_completed = 0
diameter_received = False
diameter = None
x = None
z = None

def control_slide(x_target, y_target):
    """控制二维滑台移动到目标坐标。"""
    rospy.loginfo(f"Moving slide to x: {x_target}, y: {y_target}")
    # move_cmd = f"move {x_target} {y_target} 1000 1\r\n"
    # ser.write(move_cmd)
    # rospy.loginfo(f"Sent move command: {move_cmd}") 

def control_gripper(action):
    """控制夹爪张开或闭合。"""
    rospy.loginfo(f"Gripper action: {action}")
    # 控制夹爪的实际代码
    rospy.sleep(2)  # 模拟夹爪动作需要时间
    if action == "open":
        gripper_done_callback("已张开")
    else:
        gripper_done_callback("已闭合")

bridge = CvBridge() 
def control_vehicle(error_margin):
    """控制车辆转向和靠近目标。"""
    global cmd_vel_pub, diameter, x, z, vehicle_threads_completed

    #twist_msg = Twist()

    while True:
        target_z = GRIPPER_R - diameter / 2
        
        if rospy.is_shutdown():  # 检查 ROS 是否关闭
            rospy.loginfo("Stopping vehicle movement due to ROS shutdown.")
            break  # 退出循环
        
        # 1. 检查是否需要转向
        if x > error_margin:
            rospy.loginfo("Turning vehicle right.")
            #twist_msg.angular.z = 0.05
        elif x < (0 - error_margin):
            rospy.loginfo("Turning vehicle left.")
            #twist_msg.angular.z = -0.05
        else:
            rospy.loginfo("Vehicle is aligned with the target.")
            # twist_msg.angular.z = 0.0

        # 2. 检查是否需要前后移动
        if x is None and z is None and diameter is None:  # 目标丢失
            target_z = GRIPPER_R
            rospy.logwarn("Target lost in camera view, using D435 depth data for control.")
            center_depth = get_d435_center_depth()
            if center_depth > target_z + error_margin:
                rospy.loginfo(f"Moving vehicle forward based on D435 depth: {center_depth}")
                # twist_msg.linear.x = 0.05
            elif center_depth < target_z - error_margin:
                rospy.loginfo(f"Moving vehicle backward based on D435 depth: {center_depth}")
                # twist_msg.linear.x = -0.05
            else:
                rospy.loginfo("Vehicle reached the desired z position based on D435 depth.")
                # twist_msg.linear.x = 0.0
                break
        else:  # 目标存在
            if z > target_z + error_margin:
                rospy.loginfo(f"Moving vehicle forward to: {target_z}, current z: {z}, error_margin = {error_margin}")
                # twist_msg.linear.x = 0.05
            elif z < target_z - error_margin:
                rospy.loginfo(f"Moving vehicle backward to: {target_z}, current z: {z}, error_margin = {error_margin}")
                # twist_msg.linear.x = -0.05
            else:
                rospy.loginfo("Vehicle reached the desired z position.")
                # twist_msg.linear.x = 0.0
                break  # 仅当所有目标都满足时才退出

        #cmd_vel_pub.publish(twist_msg)
        time.sleep(0.1)

        if system_state != SystemState.WAITING_FOR_ACTIONS:
            rospy.loginfo("Stopping vehicle movement due to system state change.")
            break

    vehicle_threads_completed += 1 # 应该只有一个线程完成
    check_vehicle_threads()


def control_vehicle_backward(distance):
    rospy.loginfo(f"Moving vehicle backward {distance} meters.")

    speed = -0.1
    duration = distance / abs(speed)

    #twist_msg = Twist()
    #twist_msg.linear.x = speed
    #cmd_vel_pub.publish(twist_msg)
    rospy.loginfo("Published backward command")

    time.sleep(duration)

    #twist_msg.linear.x = 0.0
    #cmd_vel_pub.publish(twist_msg)

    rospy.loginfo(f"Vehicle finished moving backward {distance} meters.")


def check_vehicle_threads():
    """检查两个车辆控制线程是否都已完成。"""
    global vehicle_threads_completed
    if vehicle_threads_completed == 1:
        vehicle_done_callback("已到达")

def control_pump(duty):
    """控制水泵PWM占空比。"""
    rospy.loginfo(f"Setting pump duty to: {duty}")
    # pump_cmd = f"pump {duty}\r\n"
    # ser.write(pump_cmd)
    #rospy.loginfo(f"Sent pump command: {pump_cmd}") 


def diameter_callback(data):
    """处理接收到的直径消息。"""
    global diameter
    diameter = data.data
    diameter_received = True
    #rospy.loginfo(f"Received diameter: {diameter} meters")

def gripper_done_callback(status):
    global system_state
    if system_state == SystemState.WAITING_FOR_ACTIONS and status == "已张开":
        rospy.loginfo("Gripper opened.")
        if vehicle_forward_thread.is_alive() == False and vehicle_turning_thread.is_alive() == False:
            system_state = SystemState.GRIPPING
            control_gripper("close") 
    elif system_state == SystemState.GRIPPING and status == "已闭合":
        rospy.loginfo("Gripper closed.")
        system_state = SystemState.PAINTING
        control_pump(100)
        rospy.sleep(5)
        control_pump(0)
        painting_done_callback("已完成")
    elif system_state == SystemState.RELEASING and status == "已张开":
        rospy.loginfo("Gripper opened, starting to move backward.")
        control_vehicle_backward(2)
        system_state = SystemState.PROCESSED


def vehicle_done_callback(status):
    global system_state
    if system_state == SystemState.WAITING_FOR_ACTIONS and status == "已到达":
        rospy.loginfo("Vehicle reached goal.")
        if gripper_thread.is_alive() == False:
            system_state = SystemState.GRIPPING
            control_gripper("close")

def painting_done_callback(status):
    global system_state
    if system_state == SystemState.PAINTING and status == "已完成":
        rospy.loginfo("Painting completed.")
        system_state = SystemState.RELEASING
        control_gripper("open")

def process_target_data(x, z):
    """处理目标坐标数据。"""
    global system_state, gripper_thread, vehicle_threads_completed
    rospy.loginfo(f"Processing target data: x={x}, z={z}")

    # 先将滑台移动到中间最上方
    control_slide(0, SLIDE_MAX_Y)

    error_margin = 0.05

    vehicle_threads_completed = 0

    gripper_thread = threading.Thread(target=control_gripper, args=("open",))
    vehicle_thread = threading.Thread(target=control_vehicle, args=(error_margin, ))  

    gripper_thread.start()
    vehicle_thread.start()

    system_state = SystemState.WAITING_FOR_ACTIONS

def search_next_target():
    """搜索下一个目标，逆时针自转直到找到有效目标。"""
    global system_state, x, z, diameter

    rospy.loginfo("Searching for the next target...")

    # 设置角速度
    angular_speed = 0.05  # 逆时针旋转

    # 创建 Twist 消息
    #twist_msg = Twist()
    #twist_msg.angular.z = angular_speed

    # 开始旋转
    #cmd_vel_pub.publish(twist_msg)

    # 循环检查 x, z, diameter 是否有效
    while x is None or z is None or diameter is None:
        time.sleep(0.3)  # 等待一小段时间

        # 检查是否需要停止搜索 (例如，系统状态改变)
        if system_state != SystemState.PROCESSED:
            rospy.loginfo("Stopping target search due to system state change.")
            #twist_msg.angular.z = 0.0
            #cmd_vel_pub.publish(twist_msg)
            return False
        if rospy.is_shutdown():  # 检查 ROS 是否关闭
            rospy.loginfo("Stopping target search due to ROS shutdown.")
            #twist_msg.angular.z = 0.0
            #cmd_vel_pub.publish(twist_msg)
            return False
    #twist_msg.angular.z = 0.0
    #cmd_vel_pub.publish(twist_msg)

    rospy.loginfo("Found a new target!")
    return True  # 搜索成功


def nearest_point_callback(data):
    """处理接收到的 nearest_point 消息。"""
    global system_state, x, z, diameter,diameter_received

    # 实时更新x、z和直径
    x = data.point.x
    z = data.point.z
    #rospy.loginfo(f"x: {x};z: {z}")

    if diameter_received:  
        if system_state == SystemState.UNPROCESSED:
            process_target_data(x, z)

def get_d435_center_depth():
    """获取 D435 视野中心点的深度数据。"""
    try:
        depth_message = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image, timeout=1) 
        
        depth_image = bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        height, width = depth_image.shape
        center_depth = depth_image[height // 2, width // 2]
        center_depth_meters = center_depth / 1000.0 
        rospy.loginfo(f"D435 center depth: {center_depth_meters} meters")
        return center_depth_meters
    except (rospy.ROSException, CvBridgeError) as e:
        rospy.logerr(f"Error getting D435 depth data: {e}")
        return None


def main():
    global cmd_vel_pub, ser

    rospy.init_node('target_processor', anonymous=True)

    # ser = serial.Serial()
    # ser.port = "/dev/ttyUSB0"
    # ser.baudrate = 115200
    # ser.timeout = 1

    # try:
    #     ser.open()
    # except serial.SerialException as e:
    #     rospy.logerr(f"Could not open serial port: {e}")
    #     return
    
    #cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/nearest_point', PointStamped, nearest_point_callback)
    rospy.Subscriber('/diameter', Float32, diameter_callback)
    # 启动后搜索
    while not rospy.is_shutdown():
        if search_next_target():
            system_state = SystemState.UNPROCESSED
        else:
            rospy.logwarn("Failed to find a new target. Restarting search...")
            system_state = SystemState.PROCESSED

    rospy.on_shutdown(shutdown_hook)

def shutdown_hook():
    """ROS 关闭时执行的回调函数。"""
    global cmd_vel_pub

    rospy.loginfo("Shutting down target_processor node...")

    # 停止车辆运动
    # twist_msg = Twist()
    # twist_msg.linear.x = 0.0
    # twist_msg.angular.z = 0.0
    # cmd_vel_pub.publish(twist_msg)


if __name__ == '__main__':
    main()
