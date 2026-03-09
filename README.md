# BlueROV2 ROS Setup Guide

This document describes how to set up **BlueROV2 with MAVROS, Qualisys, and PID control in ROS Noetic**.

---

# 1. Install MAVROS

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

BlueROV2 default IP:

```
192.168.2.2
```

---

# 3. Launch MAVROS

Run MAVROS:

```bash
roslaunch mavros apm.launch \
fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555 \
target_system_id:=1 \
target_component_id:=1
```

If MAVROS cannot connect and shows network errors, set the ROS IP:

```bash
export ROS_IP=192.168.0.170
```

You can add this to `.bashrc`:

```bash
echo "export ROS_IP=192.168.0.170" >> ~/.bashrc
```

If successful, MAVROS will start receiving **heartbeat messages**.

Check MAVROS state:

```bash
rostopic echo /mavros/state
```

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

# 5. Install Qualisys ROS Package

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
ros_qualysis/src/launch/qualisys_bauzil_bringup.launch
```

---

# 7. Transform Qualisys Data to ROS Pose

Run the transformation script:

```bash
python3 ~/bluerov2_pid/ros_qualysis/src/scripts/tf2_pose_gt_real.py
```

This converts **Qualisys motion capture data** into **ROS pose data**.

---

# 8. Launch QTM

Run:

```bash
roslaunch ros-qualisys qualisys_bauzil_bringup.launch \
server_address:=172.20.10.3 \
server_base_port:=33333
```

Make sure the **IP address and port match both Windows and Ubuntu settings**.

If successful, the rigid body should appear.

---

# 9. Launch PID Controller

```bash
roslaunch /home/bluerov2_pid/bluerov2_control/src/bluerov2_motion_control/launch/bluerov2_motion_control.launch
```

---

# 10. Publish PWM to BlueROV2

```bash
roslaunch /home/bluerov2_pid/bluerov2_control/src/mavros_pub/launch/pwm_pub.launch
```

---

# Full Workflow

1. Configure network

```
IP: 192.168.2.1
Subnet: 255.255.255.0
```

2. Launch MAVROS

```bash
roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555
```

3. Check MAVROS state

```bash
rostopic echo /mavros/state
```

4. Arm BlueROV2

```bash
rosservice call /mavros/cmd/arming "value: true"
```

5. Launch Qualisys

```bash
roslaunch ros-qualisys qualisys_bauzil_bringup.launch
```

6. Transform Qualisys data

```bash
python3 tf2_pose_gt_real.py
```

7. Launch PID controller

```bash
roslaunch bluerov2_motion_control.launch
```

8. Publish PWM

```bash
roslaunch pwm_pub.launch
```














