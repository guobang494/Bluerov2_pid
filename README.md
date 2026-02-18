Install the mavros

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins   


Setting the bluerov2 network 

To set up the BlueROV2 network, configure your topside computer with a static IP address: IP Address: 192.168.2.1  （Bluerov2 IP）
Subnet Mask: 255.255.255.0

Lanuch the mavros

roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555 target_system_id:=1 target_component_id:=1

if cannot launch the mavros and error info is about connection 
set the ROS IP 
open in terminal 
export ROS_IP=192.168.0.170 (Wifi IP)   run in every terminal or write in the bashrc

If success ,you can see the heart beats!!


Install the qulaisys

Down load the whole package 

cd ~/bluerov2_pid/ros_qualysis
export CMAKE_PREFIX_PATH=/opt/openrobots
source /opt/ros/XXXX/setup.bash

cd ~/bluerov2_pid/ros_qualysis
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


launch
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
source ./install/setup.bash
roslaunch ~/bluerov2_pid/ros_qualysis/src/launch/qualisys_bauzil_bringup.launch


and you can change the ip adress into yours
path :ros_qualysis/src/launch/qualisys_bauzil_bringup.launch


Transform quliasys date into ros data 
run 
python3  ~/bluerov2_pid/ros_qualysis/src/scripts/tf2_pose_gt_real.py


Whole process

1. launch the mavros 

roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555 target_system_id:=1 target_component_id:=1

Before that users should do the Network Setup
enter 192.168.2.1 for the IP address and 255.255.255.0 for the Subnet mask

If success 
you will see mavros heart beats

check the states
rostopic echo /mavros/state

2. arm and disarm
rosservice call /mavros/cmd/arming "value: false"
rosservice call /mavros/cmd/arming "value: true"


3. launch the QTM


roslaunch ros-qualisys qualisys_bauzil_bringup.launch     server_address:=172.20.10.3     server_base_port:=33333(set it to your own ip and port both in windows and ubuntu)

you will see that rigid body xx

launch the data transform from the QTM data to Ros data

python3  ~/bluerov2_pid/ros_qualysis/src/scripts/tf2_pose_gt_real.py



4. Launch the PID
roslauch /home/bluerov2_pid/bluerov2_control/src/bluerov2_motion_control/launch/bluerov2_motion_control.launch

5 Launch the PWM to Bluerov2
roslauch /home/bluerov2_pid/bluerov2_control/src/mavros_pub/launch/pwm_pub.launch


