cmake_minimum_required(VERSION 3.0.2)
project(add2num)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs  # Don't need message_runtime if no custom messages
)

catkin_install_python(PROGRAMS
  scripts/number1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  # ${catkin_INCLUDE_DIRS}  # You can uncomment this line if you need to include headers
)

# No need for custom C++ libraries or executables if you're only using Python
