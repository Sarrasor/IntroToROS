cmake_minimum_required(VERSION 3.0.2)
project(urdf_prober_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES urdf_robot
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control src/control.cc)
target_link_libraries(control ${catkin_LIBRARIES})