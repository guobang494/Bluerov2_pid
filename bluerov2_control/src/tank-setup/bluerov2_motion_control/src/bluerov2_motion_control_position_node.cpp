/*
    A simple implementation of a PID control with antiwindup and saturation clamping for position control.
    This code reads a desired pose, reads the current pose (from Qualisys), and produces a reference velocity. 
    
    author: Davide Grande
    date: 11/02/2026
*/


#include "bluerov2_motion_control/pid.h"


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <array>
#include <cmath>
#include <string>
#include <vector>

static bool isAngularTopic(const std::string& topic) {
  return topic.find("/angular/") != std::string::npos;
}


// This defines a structure to be reused over the 4 control loops (or control axes)
struct Axis {
  std::string group_name;     // bluerov2_motion_control_pos (param group name)
  std::string pid_ns;   // PID_u1_position, ...
  std::string meas_topic;
  std::string ref_topic;
  std::string out_topic;

  double meas{0.0};
  double ref{0.0};
  bool have_meas{false};
  bool have_ref{false};

  ros::Subscriber sub_meas;  // subscriber publisher 
  ros::Subscriber sub_ref;  // subsciber reference
  ros::Publisher pub_out;  // publisher velocity 

  SimplePID pid;
};

static void loadPid(const ros::NodeHandle& pnh, const std::string& group_name, const std::string& pid_ns, SimplePID& pid)
{
  double kp, ki, kd, umin, umax;
  bool antiwindup;

  pnh.param(group_name + "/" + pid_ns + "/Gain/Kp", kp, 0.0);
  pnh.param(group_name + "/" + pid_ns + "/Gain/Ki", ki, 0.0);
  pnh.param(group_name + "/" + pid_ns + "/Gain/Kd", kd, 0.0);
  pnh.param(group_name + "/" + pid_ns + "/Saturation/UMin", umin, -1e9);
  pnh.param(group_name + "/" + pid_ns + "/Saturation/UMax", umax,  1e9);
  pnh.param(group_name + "/" + pid_ns + "/Anti_wind_up/enable_anti_wind_up", antiwindup, true);

  pid.setGains(kp, ki, kd);
  pid.setSaturation(umin, umax);
  pid.setAntiWindup(antiwindup);
  pid.reset();

  ROS_INFO_STREAM("Loaded " << group_name << "/" << pid_ns << " Kp=" << kp << " Ki=" << ki << " Kd=" << kd
                  << " UMin=" << umin << " UMax=" << umax << " antiwindup=" << (antiwindup ? "true":"false"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bluerov2_motion_control_position_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::vector<Axis> axes = {
    {"bluerov2_motion_control_pos",   "PID_u1_position",   "/bluerov2_heavy/position/linear/x",    "/bluerov2_heavy/reference_position/linear/x",    "/bluerov2_heavy/reference_velocity/linear/x"},
    {"bluerov2_motion_control_pos",   "PID_u2_position",   "/bluerov2_heavy/position/linear/y",    "/bluerov2_heavy/reference_position/linear/y",    "/bluerov2_heavy/reference_velocity/linear/y"},
    {"bluerov2_motion_control_pos",   "PID_u3_position",   "/bluerov2_heavy/position/linear/z",    "/bluerov2_heavy/reference_position/linear/z",    "/bluerov2_heavy/reference_velocity/linear/z"},
    {"bluerov2_motion_control_pos", "PID_u4_position", "/bluerov2_heavy/position/angular/x", "/bluerov2_heavy/reference_position/angular/x",  "/bluerov2_heavy/reference_velocity/angular/x"},
    {"bluerov2_motion_control_pos", "PID_u5_position", "/bluerov2_heavy/position/angular/y", "/bluerov2_heavy/reference_position/angular/y",  "/bluerov2_heavy/reference_velocity/angular/y"},
    {"bluerov2_motion_control_pos", "PID_u6_position", "/bluerov2_heavy/position/angular/z", "/bluerov2_heavy/reference_position/angular/z",  "/bluerov2_heavy/reference_velocity/angular/z"}
  };  // TODO confirm parameter group definition!

  // Load PIDs from private params (~PID_x_position/...)
  for (auto& ax : axes) {
    loadPid(pnh, ax.group_name, ax.pid_ns, ax.pid);
    ax.pid.setAngleWrap(isAngularTopic(ax.meas_topic));
  }

  double rate_hz, sampling_time_Ts;
  pnh.param(axes[0].group_name + "/sampling_time_Ts", sampling_time_Ts, 100.0);
  rate_hz = 1/sampling_time_Ts;


  // ROS interfaces
  for (auto& ax : axes) {
    ax.pub_out = nh.advertise<std_msgs::Float64>(ax.out_topic, 1);

    ax.sub_meas = nh.subscribe<std_msgs::Float64>(
      ax.meas_topic, 1,
      [&ax](const std_msgs::Float64::ConstPtr& msg){
        ax.meas = msg->data;
        ax.have_meas = true;
      });

    ax.sub_ref = nh.subscribe<std_msgs::Float64>(
      ax.ref_topic, 1,
      [&ax](const std_msgs::Float64::ConstPtr& msg){
        ax.ref = msg->data;
        ax.have_ref = true;
      });
  }

  ros::Rate rate(rate_hz);
  ros::Time last = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    ros::Time now = ros::Time::now();
    double dt = (now - last).toSec();
    last = now;

    const bool have_linear = axes[0].have_meas && axes[0].have_ref &&
                             axes[1].have_meas && axes[1].have_ref &&
                             axes[2].have_meas && axes[2].have_ref;
    const bool have_angular_meas = axes[3].have_meas && axes[4].have_meas && axes[5].have_meas;
    const bool have_angular_ref = axes[3].have_ref && axes[4].have_ref && axes[5].have_ref;

    // Rotate linear position error to body frame, then run PID in body frame.
    if (have_linear && have_angular_meas) {
      const double roll = axes[3].meas;
      const double pitch = axes[4].meas;
      const double yaw = axes[5].meas;

      const double cphi = std::cos(roll);
      const double sphi = std::sin(roll);
      const double cth = std::cos(pitch);
      const double sth = std::sin(pitch);
      const double cps = std::cos(yaw);
      const double sps = std::sin(yaw);

      // R_bw rotates vectors from inertial/world frame to body frame (ZYX convention).
      const double r11 = cth * cps;
      const double r12 = cth * sps;
      const double r13 = -sth;
      const double r21 = sphi * sth * cps - cphi * sps;
      const double r22 = sphi * sth * sps + cphi * cps;
      const double r23 = sphi * cth;
      const double r31 = cphi * sth * cps + sphi * sps;
      const double r32 = cphi * sth * sps - sphi * cps;
      const double r33 = cphi * cth;

      const double ex = axes[0].ref - axes[0].meas;
      const double ey = axes[1].ref - axes[1].meas;
      const double ez = axes[2].ref - axes[2].meas;

      std::array<double, 3> e_lin_body{};
      e_lin_body[0] = r11 * ex + r12 * ey + r13 * ez;
      e_lin_body[1] = r21 * ex + r22 * ey + r23 * ez;
      e_lin_body[2] = r31 * ex + r32 * ey + r33 * ez;

      std::array<double, 3> u_lin_body{};
      for (int i = 0; i < 3; ++i) {
        // PID runs on body-frame position error (ref = error, meas = 0).
        u_lin_body[i] = axes[i].pid.update(e_lin_body[i], 0.0, dt);
      }

      for (int i = 0; i < 3; ++i) {
        std_msgs::Float64 out;
        out.data = u_lin_body[i];
        axes[i].pub_out.publish(out);
      }
    }

    // 1) Angular position loops produce Euler angle rates.
    // 2) Convert Euler rates [phi_dot, theta_dot, psi_dot] to body rates [p, q, r].
    if (have_angular_meas && have_angular_ref) {
      const double phi = axes[3].meas;
      const double theta = axes[4].meas;

      const double phi_dot = axes[3].pid.update(axes[3].ref, axes[3].meas, dt);
      const double theta_dot = axes[4].pid.update(axes[4].ref, axes[4].meas, dt);
      const double psi_dot = axes[5].pid.update(axes[5].ref, axes[5].meas, dt);

      const double cphi = std::cos(phi);
      const double sphi = std::sin(phi);
      const double cth = std::cos(theta);
      const double sth = std::sin(theta);

      const double p = phi_dot - sth * psi_dot;
      const double q = cphi * theta_dot + sphi * cth * psi_dot;
      const double r = -sphi * theta_dot + cphi * cth * psi_dot;

      std_msgs::Float64 out_p;
      out_p.data = p;
      axes[3].pub_out.publish(out_p);

      std_msgs::Float64 out_q;
      out_q.data = q;
      axes[4].pub_out.publish(out_q);

      std_msgs::Float64 out_r;
      out_r.data = r;
      axes[5].pub_out.publish(out_r);
    }

    rate.sleep();
  }

  return 0;
}
