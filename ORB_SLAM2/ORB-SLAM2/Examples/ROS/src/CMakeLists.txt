cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2)


# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# rosbuild_init()
# IF(NOT ROS_BUILD_TYPE)
#   SET(ROS_BUILD_TYPE Release)
# ENDIF()
# MESSAGE("Build type: " ${ROS_BUILD_TYPE})


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost COMPONENTS system)
find_package(mynteye)
find_package(mynteyed)

if(mynteye_FOUND)
   message(STATUS "MYNT-EYE-S-SDK found.")
else()
   message(STATUS "MYNT-EYE-S-SDK not found.")
endif()

if(mynteyed_FOUND)
   message(STATUS "MYNT-EYE-D-SDK found.")
else()
   message(STATUS "MYNT-EYE-D-SDK not found.")
endif()


## 依赖的ROS
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    tf
    image_transport
    cv_bridge
    )

find_package(OpenCV 3.1 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${PROJECT_SOURCE_DIR}/../../../
  ${PROJECT_SOURCE_DIR}/../../../include
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(
${Pangolin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so

)


####################################################################
####################################################################

# Node for Mono camera
add_executable(Mono
src/ros_mono.cc
)
target_link_libraries(Mono
${catkin_LIBRARIES}
${LIBS}
)


# Node for monocular camera
add_executable(mynteye_mono
src/ros_mynteye_mono.cc
)
target_link_libraries(mynteye_mono
${catkin_LIBRARIES}
${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
# add_executable(MonoAR
# src/AR/ros_mono_ar.cc
# src/AR/ViewerAR.h
# src/AR/ViewerAR.cc
# )
# target_link_libraries(MonoAR
# ${catkin_LIBRARIES}
# ${LIBS}
# )

# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${catkin_LIBRARIES}
${LIBS}
)
# Node for mynteye stereo camera

if(mynteyed_FOUND)
   add_executable(mynteye_d_stereo
   src/ros_mynteye_d_stereo.cc
   )

   target_link_libraries(mynteye_d_stereo
   ${catkin_LIBRARIES}
   ${LIBS}
   )
endif()

if(mynteye_FOUND)
   add_executable(mynteye_s_stereo
   src/ros_mynteye_s_stereo.cc
   )

   target_link_libraries(mynteye_s_stereo
   ${catkin_LIBRARIES}
   ${LIBS}
   )
endif()

# Node for mynteye rgbd camera
# if(mynteyed_FOUND)
#    add_executable(mynteye_rgbd
#    src/ros_mynteye_rgbd.cc
#    )

#    target_link_libraries(mynteye_rgbd
#    ${catkin_LIBRARIES}
#    ${LIBS}
#    )
# endif()

# Node for RGB-D camera
# add_executable(RGBD
# src/ros_rgbd.cc
# )

# target_link_libraries(RGBD
# ${catkin_LIBRARIES}
# ${LIBS}
# )

