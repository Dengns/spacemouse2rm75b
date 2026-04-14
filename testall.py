# from Robotic_Arm.rm_robot_interface import *
import time

# # 实例化RoboticArm类
# arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# # 创建机械臂连接，打印连接id
# handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
# print(handle.id)
# print(time.time())
# # print(time.time(),arm.rm_movep_follow([0,0,0.807,0,0,0]))
# print(arm.rm_movep_canfd([0,0,0.847,0,0,0], True))
# print(time.time())
# time.sleep(2)
# print(time.time())
# print(arm.rm_movep_canfd([0,0,0.848,0,0,0], True, 1, 60))
# # print(arm.rm_movep_follow([0,0,0.847,0,0,0]))
# time.sleep(2)
# print(time.time())
# arm.rm_delete_robot_arm()
from Robotic_Arm.rm_robot_interface import *

# 实例化RoboticArm类
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# 创建机械臂连接，打印连接id
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
print(handle.id)
for i in range (199):
    print(arm.rm_movep_canfd([0.26,0,0.38,3.14,0,3.017], True))
    time.sleep(0.04)
time.sleep(2)
arm.rm_delete_robot_arm()