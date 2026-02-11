#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist

# set limit
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# map to pwm
def map_to_pwm_x(n, pwm_min_x=1400, pwm_mid_x=1500, pwm_max_x=1600, deadband=0.023):
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_x
    if n >= 0:
        pwm = pwm_mid_x + n * (pwm_max_x - pwm_mid_x)
    else:
        pwm = pwm_mid_x + n * (pwm_mid_x - pwm_min_x)
    return int(clamp(int(round(pwm)), pwm_min_x, pwm_max_x))

def map_to_pwm_y(n, pwm_min_y=1400, pwm_mid_y=1500, pwm_max_y=1600, deadband=0.023):
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_y
    if n >= 0:
        pwm = pwm_mid_y + n * (pwm_max_y - pwm_mid_y)
    else:
        pwm = pwm_mid_y + n * (pwm_mid_y - pwm_min_y)
    return int(clamp(int(round(pwm)), pwm_min_y, pwm_max_y))

def map_to_pwm_z(n, pwm_min_z=1500, pwm_mid_z=1500, pwm_max_z=1500, deadband=0.023):
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_z
    if n >= 0:
        pwm = pwm_mid_z + n * (pwm_max_z - pwm_mid_z)
    else:
        pwm = pwm_mid_z + n * (pwm_mid_z - pwm_min_z)
    return int(clamp(int(round(pwm)), pwm_min_z, pwm_max_z))

def map_to_pwm_yaw(n, pwm_min_yaw=1460, pwm_mid_yaw=1500, pwm_max_yaw=1540, deadband=0.023):
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_yaw
    if n >= 0:
        pwm = pwm_mid_yaw + n * (pwm_max_yaw - pwm_mid_yaw)
    else:
        pwm = pwm_mid_yaw + n * (pwm_mid_yaw - pwm_min_yaw)
    return int(clamp(int(round(pwm)), pwm_min_yaw, pwm_max_yaw))


# define publish function
def publish_pwm_dict(pub, ch_pwm):
    msg = OverrideRCIn()
    msg.channels = [0] * 18
    for ch, pwm in ch_pwm.items():
        idx = int(ch) - 1
        if 0 <= idx < 18:
            msg.channels[idx] = int(pwm)
    pub.publish(msg)

# 全局变量存储当前力/力矩
current_force = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0, 'mz': 0.0}
force_ready = False

def force_callback(msg):
    """从DOBMPC力/力矩命令话题获取机体坐标系控制信息"""
    global current_force, force_ready
    
    # Twist消息现在携带的是力/力矩: linear=Force(N), angular=Torque(N*m)
    current_force['fx'] = msg.linear.x   # 机体X轴推力 (N)
    current_force['fy'] = msg.linear.y   # 机体Y轴推力 (N)
    current_force['fz'] = msg.linear.z   # 机体Z轴推力 (N)
    current_force['mz'] = msg.angular.z  # 绕Z轴力矩 (N*m)
    force_ready = True

# define node
def main():
    rospy.init_node("force_to_rc_override")

    # ROS话题配置 - 默认为 /bluerov2/cmd_vel (复用旧名) 或 /bluerov2/cmd_force
    force_topic = rospy.get_param('~force_topic', '/bluerov2/cmd_vel')

    # parameters (从launch文件或默认值读取)
    pwm_mid = 1500  # 中性PWM值
    deadband = 0.023
    
    # 最大推力/力矩参数 (用于归一化)
    # 假设单推进器最大推力 ~30-50N，整机合力 X/Y ~100N, Z ~100N, Mz ~20Nm
    # 这些值可以根据实际机器人能力调整
    max_force_x = rospy.get_param('~max_force_x', 10.0)  # N
    max_force_y = rospy.get_param('~max_force_y', 10.0)  # N 
    max_force_z = rospy.get_param('~max_force_z', 0.0)  # N 
    max_torque_z = rospy.get_param('~max_torque_z', 1.0) # N*m
    
    ch_forward, ch_lateral, ch_vertical, ch_yaw = 5, 6, 3, 4    # channel define
    play_rate_hz = 20.0

    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    
    # 订阅MPC力/力矩命令话题
    force_sub = rospy.Subscriber(force_topic, Twist, force_callback, queue_size=10)
    
    # 等待ROS数据
    rospy.loginfo("wait for force data...") 
    while not rospy.is_shutdown() and not force_ready:
        rospy.sleep(0.1)
        
    if rospy.is_shutdown():
        return
        
    rospy.loginfo("开始实时推力控制...")
    rate = rospy.Rate(play_rate_hz)
    
    # 实时处理
    while not rospy.is_shutdown():
        # 获取当前机体坐标系力/力矩
        fx = current_force['fx']
        fy = current_force['fy']
        fz = current_force['fz']
        mz = current_force['mz']


        
        # 归一化 (-1.0 到 1.0)
        nx = clamp(fx / max_force_x, -1.0, 1.0)
        ny = clamp(fy / max_force_y, -1.0, 1.0)
        nz = clamp(fz / max_force_z, -1.0, 1.0)
        nr = clamp(mz / max_torque_z, -1.0, 1.0)

        # 映射为 PWM 信号
        # 注意通道对应关系：
        # ch_forward (5) -> nx (Surge)
        # ch_lateral (6) -> ny (Sway)
        # ch_vertical (3) -> nz (Heave)
        # ch_yaw (4) -> nr (Yaw)
        
        
        # 假设这里按照标准 X=Forward, Y=Left
        pwm_forward = map_to_pwm_x(nx)    
        pwm_lateral = map_to_pwm_y(ny)    
        pwm_vertical = map_to_pwm_z(nz)   
        pwm_yaw = map_to_pwm_yaw(nr)      

        # 发布PWM
        publish_pwm_dict(pub, {
            ch_forward: pwm_forward,
            ch_lateral: pwm_lateral,
            ch_vertical: pwm_vertical,
            ch_yaw: pwm_yaw
        })
        
        # 定期打印调试信息
        if rospy.get_time() % 2.0 < 0.1:  # 每2秒打印一次
            rospy.loginfo_throttle(2.0, 
                f"MPC力/力矩: Fx={fx:.1f}, Fy={fy:.1f}, Fz={fz:.1f}, Mz={mz:.1f}")
            rospy.loginfo_throttle(2.0,
                f"归一化: nx={nx:.3f}, ny={ny:.3f}, nz={nz:.3f}, nr={nr:.3f}")
            rospy.loginfo_throttle(2.0,
                f"PWM输出: ch{ch_forward}={pwm_forward}, ch{ch_lateral}={pwm_lateral}, ch{ch_vertical}={pwm_vertical}, ch{ch_yaw}={pwm_yaw}")

        rate.sleep()

    # 关闭时发送中性PWM
    rospy.loginfo("发送中性PWM信号...")
    for _ in range(5):
        publish_pwm_dict(pub, {
            ch_forward: pwm_mid,
            ch_lateral: pwm_mid,
            ch_vertical: pwm_mid,
            ch_yaw: pwm_mid
        })
        rospy.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
