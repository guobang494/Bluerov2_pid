cmake_minimum_required(VERSION 3.0.2)
project(guidance_law)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/guidance_law_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch configs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
