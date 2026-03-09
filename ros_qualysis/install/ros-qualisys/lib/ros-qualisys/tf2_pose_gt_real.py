#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_msgs.msg import TFMessage
import numpy as np
import math

class TF2PoseGroundTruth:
    def __init__(self):
        rospy.init_node('tf2_pose_gt_real', anonymous=True)
        
        # Parameters
        self.target_frame = rospy.get_param('~child_frame_id', 'BlueROV2H')  # 要跟踪的刚体名称
        self.reference_frame = rospy.get_param('~frame_id', 'mocap')     # 参考坐标系
        self.publish_rate = rospy.get_param('~publish_rate', 20.0)       # 发布频率 Hz
        self.pose_topic = rospy.get_param('~pose_topic', '/bluerov2_heavy')  # 话题前缀
        
        # Subscribe to /tf topic from ros_qualisys
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        
        # Publisher for ground truth pose (Odometry format, 保留兼容)
        self.pose_pub = rospy.Publisher(self.pose_topic, Odometry, queue_size=10)
        
        # 8 个 Float64 publishers — 对接 bluerov2_motion_control
        self.pub_pos_x   = rospy.Publisher(self.pose_topic + '/position/linear/x',  Float64, queue_size=10)
        self.pub_pos_y   = rospy.Publisher(self.pose_topic + '/position/linear/y',  Float64, queue_size=10)
        self.pub_pos_z   = rospy.Publisher(self.pose_topic + '/position/linear/z',  Float64, queue_size=10)
        self.pub_pos_yaw = rospy.Publisher(self.pose_topic + '/position/angular/z', Float64, queue_size=10)

        self.pub_vel_x   = rospy.Publisher(self.pose_topic + '/velocity/linear/x',  Float64, queue_size=10)
        self.pub_vel_y   = rospy.Publisher(self.pose_topic + '/velocity/linear/y',  Float64, queue_size=10)
        self.pub_vel_z   = rospy.Publisher(self.pose_topic + '/velocity/linear/z',  Float64, queue_size=10)
        self.pub_vel_yaw = rospy.Publisher(self.pose_topic + '/velocity/angular/z', Float64, queue_size=10)
        
        # Variables for velocity calculation
        self.last_pose = None
        self.last_yaw = None
        self.last_time = None
        self.current_transform = None
        
        # Rate control
        self.rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo(f"TF2 Pose Ground Truth Node started")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        rospy.loginfo(f"Reference frame: {self.reference_frame}")
        rospy.loginfo(f"Subscribing to: /tf")
        rospy.loginfo(f"Publishing Odometry to: {self.pose_topic}")
        rospy.loginfo(f"Publishing Float64 to: {self.pose_topic}/position/... and {self.pose_topic}/velocity/...")
        rospy.loginfo(f"Publish rate: {self.publish_rate} Hz")
    
    def tf_callback(self, tf_msg):
        """Callback for the /tf topic subscriber."""
        rospy.loginfo_throttle(5.0, f"Received tf message with {len(tf_msg.transforms)} transforms")
        
        # Find the transform from the reference frame to the target frame
        for transform in tf_msg.transforms:
            rospy.loginfo_throttle(5.0, f"Transform: frame_id={transform.header.frame_id}, child_frame_id={transform.child_frame_id}")
            
            if transform.header.frame_id == self.reference_frame and transform.child_frame_id == self.target_frame:
                self.current_transform = transform
                rospy.loginfo_throttle(5.0, f"Found matching transform for {self.target_frame}!")
                break
        else:
            rospy.logwarn_throttle(5.0, f"No matching transform found. Looking for: frame_id={self.reference_frame}, child_frame_id={self.target_frame}")
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """从四元数提取偏航角 yaw (rad)"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_multiply(self, q1, q2):
        """四元数乘法 [x, y, z, w] 格式"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
    
    def calculate_velocity(self, current_pose, current_yaw, current_time):
        """计算线速度和角速度"""
        linear_vel = [0.0, 0.0, 0.0]
        angular_vel = [0.0, 0.0, 0.0]
        
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                # 计算线速度
                linear_vel[0] = (current_pose.position.x - self.last_pose.position.x) / dt
                linear_vel[1] = (current_pose.position.y - self.last_pose.position.y) / dt
                linear_vel[2] = (current_pose.position.z - self.last_pose.position.z) / dt
                
                # 正确的角速度计算：使用四元数微分
                q_curr = np.array([current_pose.orientation.x, 
                                   current_pose.orientation.y, 
                                   current_pose.orientation.z, 
                                   current_pose.orientation.w])
                q_last = np.array([self.last_pose.orientation.x, 
                                   self.last_pose.orientation.y, 
                                   self.last_pose.orientation.z, 
                                   self.last_pose.orientation.w])
                
                # 确保四元数符号一致（避免 q 和 -q 表示同一旋转的问题）
                if np.dot(q_curr, q_last) < 0:
                    q_last = -q_last
                
                # 计算相对旋转四元数: q_rel = q_curr * q_last^(-1)
                q_last_inv = np.array([-q_last[0], -q_last[1], -q_last[2], q_last[3]])
                q_rel = self.quaternion_multiply(q_curr, q_last_inv)
                
                # 从相对四元数提取角速度 (axis-angle 方法)
                angle = 2.0 * np.arccos(np.clip(q_rel[3], -1.0, 1.0))
                if angle > 1e-6:
                    sin_half_angle = np.sin(angle / 2.0)
                    axis = q_rel[:3] / sin_half_angle
                    angular_vel = (axis * angle / dt).tolist()
                else:
                    angular_vel = [0.0, 0.0, 0.0]
        
        # yaw 角速度单独用差分法计算（更稳定，用于 Float64 输出）
        yaw_rate = 0.0
        if self.last_yaw is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                yaw_rate = self.normalize_angle(current_yaw - self.last_yaw) / dt
        
        return linear_vel, angular_vel, yaw_rate
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                # Check if we have received tf data
                if self.current_transform is None:
                    rospy.logwarn_throttle(1.0, f"Waiting for tf data for frame: {self.target_frame}")
                    self.rate.sleep()
                    continue
                
                # 创建 Odometry 消息
                odom_msg = Odometry()
                
                # 填充消息头
                odom_msg.header.stamp = self.current_transform.header.stamp
                odom_msg.header.frame_id = self.reference_frame
                odom_msg.child_frame_id = self.target_frame
                
                # 填充位置信息
                odom_msg.pose.pose.position.x = self.current_transform.transform.translation.x
                odom_msg.pose.pose.position.y = self.current_transform.transform.translation.y
                odom_msg.pose.pose.position.z = self.current_transform.transform.translation.z
                
                # 填充姿态信息
                odom_msg.pose.pose.orientation.x = self.current_transform.transform.rotation.x
                odom_msg.pose.pose.orientation.y = self.current_transform.transform.rotation.y
                odom_msg.pose.pose.orientation.z = self.current_transform.transform.rotation.z
                odom_msg.pose.pose.orientation.w = self.current_transform.transform.rotation.w
                
                # 从四元数提取 yaw
                yaw = self.quaternion_to_yaw(
                    odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w
                )
                
                # 计算速度
                current_time = self.current_transform.header.stamp
                linear_vel, angular_vel, yaw_rate = self.calculate_velocity(
                    odom_msg.pose.pose, yaw, current_time
                )
                
                # 填充速度信息
                odom_msg.twist.twist.linear.x = linear_vel[0]
                odom_msg.twist.twist.linear.y = linear_vel[1]
                odom_msg.twist.twist.linear.z = linear_vel[2]
                odom_msg.twist.twist.angular.x = angular_vel[0]
                odom_msg.twist.twist.angular.y = angular_vel[1]
                odom_msg.twist.twist.angular.z = angular_vel[2]
                
                # 设置协方差矩阵
                odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                          0, 0.01, 0, 0, 0, 0,
                                          0, 0, 0.01, 0, 0, 0,
                                          0, 0, 0, 0.01, 0, 0,
                                          0, 0, 0, 0, 0.01, 0,
                                          0, 0, 0, 0, 0, 0.01]
                
                odom_msg.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                           0, 0.1, 0, 0, 0, 0,
                                           0, 0, 0.1, 0, 0, 0,
                                           0, 0, 0, 0.1, 0, 0,
                                           0, 0, 0, 0, 0.1, 0,
                                           0, 0, 0, 0, 0, 0.1]
                
                # 发布 Odometry (保留兼容)
                self.pose_pub.publish(odom_msg)
                
                # 发布 8 个 Float64 话题 — 对接 bluerov2_motion_control
                self.pub_pos_x.publish(Float64(data=odom_msg.pose.pose.position.x))
                self.pub_pos_y.publish(Float64(data=odom_msg.pose.pose.position.y))
                self.pub_pos_z.publish(Float64(data=odom_msg.pose.pose.position.z))
                self.pub_pos_yaw.publish(Float64(data=yaw))

                self.pub_vel_x.publish(Float64(data=linear_vel[0]))
                self.pub_vel_y.publish(Float64(data=linear_vel[1]))
                self.pub_vel_z.publish(Float64(data=linear_vel[2]))
                self.pub_vel_yaw.publish(Float64(data=yaw_rate))
                
                # 更新历史数据用于速度计算
                self.last_pose = odom_msg.pose.pose
                self.last_yaw = yaw
                self.last_time = current_time
                
                # 日志输出
                rospy.loginfo_throttle(2.0, 
                    f"Pos: x={odom_msg.pose.pose.position.x:.3f} y={odom_msg.pose.pose.position.y:.3f} "
                    f"z={odom_msg.pose.pose.position.z:.3f} yaw={math.degrees(yaw):.1f}deg")
                rospy.loginfo_throttle(2.0,
                    f"Vel: vx={linear_vel[0]:.3f} vy={linear_vel[1]:.3f} "
                    f"vz={linear_vel[2]:.3f} vyaw={yaw_rate:.3f}")
                
            except Exception as e:
                rospy.logerr(f"Unexpected error: {str(e)}")
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TF2PoseGroundTruth()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF2 Pose Ground Truth node terminated.")
    except Exception as e:
        rospy.logerr(f"Failed to start TF2 Pose Ground Truth node: {str(e)}")
