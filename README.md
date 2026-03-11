# BlueROV2 ROS Setup Guide

This document describes how to set up **BlueROV2 with MAVROS, Qualisys, and PID control in ROS Noetic**.

---

# 1. Install MAVROS (If you use docker we provided then skip this)

Install MAVROS and related packages.

```bash
sudo apt-get update

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

chmod a+x install_geographiclib_datasets.sh

./install_geographiclib_datasets.sh
```

Install RQT tools:

```bash
sudo apt-get install \
ros-noetic-rqt \
ros-noetic-rqt-common-plugins \
ros-noetic-rqt-robot-plugins
```

---

# 2. BlueROV2 Network Setup

Configure the **topside computer** with a static IP:

```
IP Address: 192.168.2.1
Subnet Mask: 255.255.255.0
```
You can alos see the official instruction for Bluerov2 which the link is locatied in 
https://bluerobotics.com/learn/bluerov2-software-setup-r3-and-older/#software-introduction


# 3. Launch MAVROS

Run MAVROS:

```bash
roslaunch mavros apm.launch \
fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555 \
target_system_id:=1 \
target_component_id:=1
```


If successful, MAVROS will start receiving **heartbeat messages**.

Check MAVROS state:

```bash
rostopic echo /mavros/state
```
You can see the status like this 




---

# 4. Arm and Disarm BlueROV2

Disarm:

```bash
rosservice call /mavros/cmd/arming "value: false"
```

Arm:

```bash
rosservice call /mavros/cmd/arming "value: true"
```

---

# 5. Install Qualisys ROS Package (If you use docker we provided then skip this)

Download the package into:

```
~/bluerov2_pid/ros_qualysis
```

Setup environment:

```bash
cd ~/bluerov2_pid/ros_qualysis

export CMAKE_PREFIX_PATH=/opt/openrobots

source /opt/ros/noetic/setup.bash
```

Build the workspace:

```bash
cd ~/bluerov2_pid/ros_qualysis

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

# 6. Launch Qualisys ROS Node

Load environment:

```bash
source /opt/ros/noetic/setup.bash

export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH

source ./install/setup.bash
```

Launch:

```bash
roslaunch ~/bluerov2_pid/ros_qualysis/src/launch/qualisys_bauzil_bringup.launch
```

If necessary, modify the server IP address in:

```
Roslaunch ros_qualysis/src/launch/qualisys_bauzil_bringup.launch server_address:=xxx.xx.xx.x     server_base_port:=xxxxx
```
If success ,if you will see 

---

# 7. Transform Qualisys Data to ROS Pose

Run the transformation script:

```bash
python3 ~/bluerov2_pid/ros_qualysis/src/scripts/tf2_pose_gt_real.py
```

This converts **Qualisys motion capture data** into **ROS pose data**.


---

# 8. Launch PID Controller

```bash
roslaunch /home/bluerov2_pid/bluerov2_control/src/bluerov2_motion_control/launch/bluerov2_motion_control.launch
```

# 9. Launch Guidance_law
```bash
roslaunch ~/bluerov2_pid/bluerov2_control/src/tank-setup/guidance_law/launch/guidance_law.launch
```
You can change the waypoint in the file 
```bash
~/bluerov2_pid/bluerov2_control/src/tank-setup/guidance_law/configs/guidance_params.yaml
```

---

# 10. Publish PWM to BlueROV2

```bash
roslaunch ~/bluerov2_pid/bluerov2_control/src/mavros_pub/launch/pwm_pub.launch
```

---

# Quick  Launch Workflow (Make sure you have seeting all parts)




### 1. MAVROS Launch & Status
```bash
roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555
rostopic echo /mavros/state
```

### 2. Arm / Disarm
```bash
# Arm
rosservice call /mavros/cmd/arming "value: true"
# Disarm
rosservice call /mavros/cmd/arming "value: false"
```

### 3. Qualisys MoCap System
```bash
roslaunch ~/bluerov2_pid/ros_qualysis/src/launch/qualisys_bauzil_bringup.launch
python3 ~/bluerov2_pid/ros_qualysis/src/scripts/tf2_pose_gt_real.py
```

### 4. Control, Guidance & PWM
```bash
roslaunch ~/bluerov2_pid/bluerov2_control/src/bluerov2_motion_control/launch/bluerov2_motion_control.launch
roslaunch ~/bluerov2_pid/bluerov2_control/src/tank-setup/guidance_law/launch/guidance_law.launch
roslaunch ~/bluerov2_pid/bluerov2_control/src/mavros_pub/launch/pwm_pub.launch
```
















