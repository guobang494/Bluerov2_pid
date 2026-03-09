#!/usr/bin/env python
import rospy
import yaml
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class GuidanceLawNode:

    def __init__(self):
        rospy.init_node("guidance_law_node")

        # Load YAML file
        config_file = rospy.get_param("~config_file")
        with open(config_file, 'r') as f:
            params = yaml.safe_load(f)

        self.path_waypoint_following = params["path_waypoint_following"]
        self.gamma = params["gamma"]
        self.waypoints = params["waypoints"]
        self.waypoint_cycling_active = params["waypoint_cycling_active"]
        self.max_position_error = float(params.get("max_position_error", 1.0))  # [m]
        self.max_angle_error_deg = float(params.get("max_angle_error_deg", 20.0))  # [deg]
        self.max_angle_error = math.radians(self.max_angle_error_deg)  # [rad]

        self.index = 0  # current waypoint index

        # Current state
        self.pos_x_gazebo = 0.0
        self.pos_y_gazebo = 0.0
        self.pos_z_gazebo = 0.0
        self.roll_gazebo = 0.0
        self.pitch_gazebo = 0.0
        self.yaw_gazebo = 0.0        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Publishers: reference position & orientation
        self.pub_ref_x = rospy.Publisher("/bluerov2_heavy/reference_position/linear/x", Float64, queue_size=10)
        self.pub_ref_y = rospy.Publisher("/bluerov2_heavy/reference_position/linear/y", Float64, queue_size=10)
        self.pub_ref_z = rospy.Publisher("/bluerov2_heavy/reference_position/linear/z", Float64, queue_size=10)
        self.pub_ref_roll  = rospy.Publisher("/bluerov2_heavy/reference_position/angular/x", Float64, queue_size=10)
        self.pub_ref_pitch = rospy.Publisher("/bluerov2_heavy/reference_position/angular/y", Float64, queue_size=10)
        self.pub_ref_yaw   = rospy.Publisher("/bluerov2_heavy/reference_position/angular/z", Float64, queue_size=10)

        # Publishers: diagnostic topics
        self.pub_next_target = rospy.Publisher("/guidance_law_variables/next_target", Float64MultiArray, queue_size=10)
        self.pub_distance = rospy.Publisher("/guidance_law_variables/distance_to_target", Float64, queue_size=10)

        # Subscribers: state
        rospy.Subscriber("/bluerov2_heavy/position/linear/x", Float64, self.cb_pos_x)
        rospy.Subscriber("/bluerov2_heavy/position/linear/y", Float64, self.cb_pos_y)
        rospy.Subscriber("/bluerov2_heavy/position/linear/z", Float64, self.cb_pos_z)
        rospy.Subscriber("/bluerov2_heavy/position/angular/x", Float64, self.cb_roll)
        rospy.Subscriber("/bluerov2_heavy/position/angular/y", Float64, self.cb_pitch)
        rospy.Subscriber("/bluerov2_heavy/position/angular/z", Float64, self.cb_yaw)

        rospy.loginfo("[guidance_law] Node started.")
        self.loop()

    @staticmethod
    def wrap_to_pi(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # --- Callbacks ---
    def cb_pos_x(self, msg): self.pos_x_gazebo = msg.data
    def cb_pos_y(self, msg): self.pos_y_gazebo = msg.data
    def cb_pos_z(self, msg): self.pos_z_gazebo = msg.data
    def cb_roll(self, msg):  self.roll = msg.data
    def cb_pitch(self, msg): self.pitch = msg.data
    def cb_yaw(self, msg):   self.yaw = msg.data

    # --- Main control loop ---
    def loop(self):
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():

            # TODO careful
            self.pos_x = self.pos_x_gazebo
            self.pos_y = self.pos_y_gazebo
            self.pos_z = self.pos_z_gazebo
            
            target = self.waypoints[self.index]
            tx, ty, tz = target

            # Compute distance to the next waypoint
            # dist = math.sqrt((tx - self.pos_x)**2 +
            #                  (ty - self.pos_y)**2 +
            #                  (tz - self.pos_z)**2) TODO restore

            dist = math.sqrt((tx - self.pos_x)**2 +
                             (ty - self.pos_y)**2)

            # Publish diagnostic distance
            self.pub_distance.publish(dist)

            # Switch waypoint when the distance is < threshold
            if dist < self.gamma:
                if self.index < len(self.waypoints) - 1:
                    self.index += 1
                elif self.waypoint_cycling_active:
                    self.index = 0

                target = self.waypoints[self.index]
                tx, ty, tz = target

            # Simple moving target: cap position error to max_position_error.
            dx = tx - self.pos_x
            dy = ty - self.pos_y
            dz = tz - self.pos_z
            dnorm = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dnorm > self.max_position_error and dnorm > 1e-9:
                s = self.max_position_error / dnorm
                tx_ref = self.pos_x + s * dx
                ty_ref = self.pos_y + s * dy
                tz_ref = self.pos_z + s * dz
            else:
                tx_ref, ty_ref, tz_ref = tx, ty, tz

            # Compute attitude toward moving target.
            dxr = tx_ref - self.pos_x
            dyr = ty_ref - self.pos_y
            dzr = tz_ref - self.pos_z
            hr = math.sqrt(dxr*dxr + dyr*dyr)

            if hr > 1e-9:
                yaw_des = math.atan2(dyr, dxr)
            else:
                yaw_des = self.yaw
            pitch_des = math.atan2(dzr, hr)
            roll_des = 0.0

            # Cap commanded angle error to +/- max_angle_error on each axis.
            e_roll = self.wrap_to_pi(roll_des - self.roll)
            e_pitch = self.wrap_to_pi(pitch_des - self.pitch)
            e_yaw = self.wrap_to_pi(yaw_des - self.yaw)

            e_roll = max(-self.max_angle_error, min(self.max_angle_error, e_roll))
            e_pitch = max(-self.max_angle_error, min(self.max_angle_error, e_pitch))
            e_yaw = max(-self.max_angle_error, min(self.max_angle_error, e_yaw))

            ref_roll = self.wrap_to_pi(self.roll + e_roll)
            ref_pitch = self.wrap_to_pi(self.pitch + e_pitch)
            ref_yaw = self.wrap_to_pi(self.yaw + e_yaw)

            # Publish reference position
            self.pub_ref_x.publish(tx_ref)
            self.pub_ref_y.publish(ty_ref)
            self.pub_ref_z.publish(tz_ref)

            # Publish reference orientation
            self.pub_ref_roll.publish(ref_roll)
            self.pub_ref_pitch.publish(ref_pitch)
            self.pub_ref_yaw.publish(ref_yaw)

            # Publish next target waypoint
            msg = Float64MultiArray()
            msg.data = [tx_ref, ty_ref, tz_ref, ref_roll, ref_pitch, ref_yaw]
            self.pub_next_target.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        GuidanceLawNode()
    except rospy.ROSInterruptException:
        pass
