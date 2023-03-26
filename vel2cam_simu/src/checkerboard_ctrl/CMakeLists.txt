cmake_minimum_required(VERSION 3.0.2)
project(checkerboard_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(checkerboard_ctrl SHARED src/checkerbor_ctrl.cc)
target_link_libraries(checkerboard_ctrl 
${catkin_INCLUDE_DIRS}
${GAZEBO_LIBRARIES})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES checkerboard_ctrl
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
