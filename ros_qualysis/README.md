# ros_qualisys
ROS wrapper around the https://github.com/qualisys/qualisys_cpp_sdk package


Installation instructions at: 
https://gepgitlab.laas.fr/gepetto/ros-qualisys



Intiall

mkdir -p workspace/src
cd workspace/src
git clone --recursive git@github.com:Gepetto/ros-qualisys.git or
git clone git@gitlab.laas.fr:gepetto/ros-qualisys.git

export CMAKE_PREFIX_PATH=/opt/openrobots
source /opt/ros/XXXX/setup.bash
cd workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release





launch
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
source ./install/setup.bash
roslaunch ros-qualisys qualisys_bauzil_bringup.launch



 connect with QTM
 
roslaunch ros-qualisys qualisys_bauzil_bringup.launch     server_address:=172.20.10.3     server_base_port:=33333
    
    
    
    
 connect
 
 hardware connect
 qualisys connetct  QIM(windows) by usb
 same IP(wifi connect)
 
 software 
 windows check IP  ipconfig
ubuntu check IP     ip a
QTM setting

check projet options real time server
check enable real timeoutput
choose TCP
set port 2222
save

Open: Control Panel → System and Security → Windows Defender Firewall → Advanced settings
Create inbound rule:
Type: Port
Protocol: TCP
Port: 22222
Action: Allow connection
Profile: Private Network


check lunix


ping 192.168.0.100
nc -zv IP 22222    test if success

roslaunch

check QualisysToRos::connect(): Connected to QTM server

if success
QualisysToRos::connect(): Connected to QTM server






























