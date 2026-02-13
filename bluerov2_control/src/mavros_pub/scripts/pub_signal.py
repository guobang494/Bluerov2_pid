#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64

# set limit
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
force_ready = {'fx': False, 'fy': False, 'fz': False, 'mz': False}

def cb_fx(msg):
    current_force['fx'] = msg.data
    force_ready['fx'] = True

def cb_fy(msg):
    current_force['fy'] = msg.data
    force_ready['fy'] = True

def cb_fz(msg):
    current_force['fz'] = msg.data
    force_ready['fz'] = True

def cb_mz(msg):
    current_force['mz'] = msg.data
    force_ready['mz'] = True


# define node
def main():
    rospy.init_node("force_to_rc_override")

    # 话题配置 — 对接 bluerov2_motion_control 速度环输出
    topic_fx = rospy.get_param('~topic_fx', '/bluerov2_heavy/cmd_velocity/linear/x')
    topic_fy = rospy.get_param('~topic_fy', '/bluerov2_heavy/cmd_velocity/linear/y')
    topic_fz = rospy.get_param('~topic_fz', '/bluerov2_heavy/cmd_velocity/linear/z')
    topic_mz = rospy.get_param('~topic_mz', '/bluerov2_heavy/cmd_velocity/angular/z')

    # 最大推力/力矩参数 (用于归一化)
    max_force_x = rospy.get_param('~max_force_x', 20.0)   # N
    max_force_y = rospy.get_param('~max_force_y', 20.0)   # N
    max_force_z = rospy.get_param('~max_force_z', 0.0)    # N  (0 表示 Z 轴不控制)
    max_torque_z = rospy.get_param('~max_torque_z', 1.0)  # N*m

    pwm_mid = 1500
    ch_forward, ch_lateral, ch_vertical, ch_yaw = 5, 6, 3, 4
    play_rate_hz = 20.0

    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)

    # 订阅 4 个 Float64 话题 — 对接 bluerov2_motion_control velocity_node 输出
    rospy.Subscriber(topic_fx, Float64, cb_fx, queue_size=10)
    rospy.Subscriber(topic_fy, Float64, cb_fy, queue_size=10)
    rospy.Subscriber(topic_fz, Float64, cb_fz, queue_size=10)
    rospy.Subscriber(topic_mz, Float64, cb_mz, queue_size=10)

    # 等待任意一个轴的数据就绪 (方便逐轴调试PID)
    rospy.loginfo("Waiting for force commands from bluerov2_motion_control (any axis)...")
    while not rospy.is_shutdown():
        if any(force_ready.values()):
            break
        rospy.sleep(0.1)

    if rospy.is_shutdown():
        return

    ready_axes = [k for k, v in force_ready.items() if v]
    rospy.loginfo("Force data received on axes: %s. Starting PWM control loop.", ready_axes)
    rate = rospy.Rate(play_rate_hz)

    while not rospy.is_shutdown():
        fx = current_force['fx']
        fy = current_force['fy']
        fz = current_force['fz']
        mz = current_force['mz']

        # 归一化 (-1.0 到 1.0)
        nx = clamp(fx / max_force_x, -1.0, 1.0) if max_force_x != 0.0 else 0.0
        ny = clamp(fy / max_force_y, -1.0, 1.0) if max_force_y != 0.0 else 0.0
        nz = clamp(fz / max_force_z, -1.0, 1.0) if max_force_z != 0.0 else 0.0
        nr = clamp(mz / max_torque_z, -1.0, 1.0) if max_torque_z != 0.0 else 0.0

        # 映射为 PWM 信号
        # ch_forward (5) -> nx (Surge)
        # ch_lateral (6) -> ny (Sway)
        # ch_vertical (3) -> nz (Heave)
        # ch_yaw (4) -> nr (Yaw)
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
        rospy.loginfo_throttle(2.0,
            f"Force: Fx={fx:.2f} Fy={fy:.2f} Fz={fz:.2f} Mz={mz:.2f}")
        rospy.loginfo_throttle(2.0,
            f"Norm:  nx={nx:.3f} ny={ny:.3f} nz={nz:.3f} nr={nr:.3f}")
        rospy.loginfo_throttle(2.0,
            f"PWM:   ch{ch_forward}={pwm_forward} ch{ch_lateral}={pwm_lateral} ch{ch_vertical}={pwm_vertical} ch{ch_yaw}={pwm_yaw}")

        rate.sleep()

    # 关闭时发送中性PWM
    rospy.loginfo("Sending neutral PWM...")
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
