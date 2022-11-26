cmake_minimum_required(VERSION 2.8.3)
project(pose_ekf)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -Ofast -Wall -fopenmp")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
# find_package(GeographicLib  REQUIRED)

find_package(catkin REQUIRED
  message_generation
  message_filters
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  hector_uav_msgs
  roslib
)

set(GeographicLib_DIR  /usr/local/include/GeographicLib)

include_directories(
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${GeographicLib_INCLUDE_DIRS}
) 

# add_definitions(${GeographicLib_DEFINITIONS})



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_ekf
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

add_executable(pose_ekf src/pose_ekf_node.cpp src/pose_ekf.cpp src/conversion.cpp)

target_link_libraries(pose_ekf
		${catkin_LIBRARIES}
    ${EIGEN_LIBRARIES}
    # ${GeographicLib_LIBRARIES}
		)


