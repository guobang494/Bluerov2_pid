/*
    A simple implementation of a PID control with antiwindup and saturation clamping for position control.
    This code reads a desired velocity, reads the current velocity (from Qualisys), and produces a reference force command. 
    
    author: Davide Grande
    date: 11/02/2026
*/



#include "bluerov2_motion_control/pid.h"


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>


// This defines a structure to be reused over the 4 control loops (or control axes)
struct Axis {
  std::string group_name;     // bluerov2_motion_control_vel (param group name)
  std::string pid_ns;   // PID_u1_velocity, ...
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
  ros::init(argc, argv, "bluerov2_motion_control_velocity_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::vector<Axis> axes = {
    {"bluerov2_motion_control_vel",   "PID_u1_velocity",   "/bluerov2_heavy/velocity/linear/x",    "/bluerov2_heavy/reference_velocity/linear/x",    "/bluerov2_heavy/cmd_velocity/linear/x"},
    {"bluerov2_motion_control_vel",   "PID_u2_velocity",   "/bluerov2_heavy/velocity/linear/y",    "/bluerov2_heavy/reference_velocity/linear/y",    "/bluerov2_heavy/cmd_velocity/linear/y"},
    {"bluerov2_motion_control_vel",   "PID_u3_velocity",   "/bluerov2_heavy/velocity/linear/z",    "/bluerov2_heavy/reference_velocity/linear/z",    "/bluerov2_heavy/cmd_velocity/linear/z"},
    {"bluerov2_motion_control_vel", "PID_u4_velocity", "/bluerov2_heavy/velocity/angular/z", "/bluerov2_heavy/reference_velocity/angular/z",  "/bluerov2_heavy/cmd_velocity/angular/z"}
  };  // TODO confirm parameter group definition!

  double rate_hz, sampling_time_Ts;
  pnh.param(axes[0].group_name + "/sampling_time_Ts", sampling_time_Ts, 100.0);
  rate_hz = 1/sampling_time_Ts;
  
  // Param to enable/disable the vehicle motion control
  bool enable_motion_control;
  pnh.param(axes[0].group_name + "/enable_motion_control", enable_motion_control, false);
  
  // When the above enable_motion_control=false, then pwm_zero_thrust will be sent as thruster request
  double pwm_zero_thrust; 
  pnh.param(axes[0].group_name + "/pwm_zero_thrust", pwm_zero_thrust, 1500.0);


  // Load PIDs from private params
  for (auto& ax : axes) loadPid(pnh, ax.group_name, ax.pid_ns, ax.pid);

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

    for (auto& ax : axes) {
      if (!ax.have_meas || !ax.have_ref) continue;

      // output is reference velocity
      double u = ax.pid.update(ax.ref, ax.meas, dt);

      std_msgs::Float64 out;

      // enable/disable motion control
      if (enable_motion_control){
        out.data = u;
      } else {
        out.data = pwm_zero_thrust;
      }
      ax.pub_out.publish(out);
    }

    rate.sleep();
  }

  return 0;
}
