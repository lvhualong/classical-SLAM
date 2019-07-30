cmake_minimum_required(VERSION 2.8.3)
project(vins_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(mynteye QUIET)
find_package(mynteyed QUIET)  ## 小觅深度相机 SDK产生的库

if(mynteye_FOUND)
   message(STATUS "MYNT-EYE-S-SDK found.")
   if(${mynteye_VERSION} VERSION_LESS "2.3.1")
      message(WARNING "MYNT-EYE-S-SDK VERSION: ${mynteye_VERSION}, camera calib info can't be identified when the sdk version is small than 2.3.1")
   endif()
else()
   message(STATUS "MYNT-EYE-S-SDK not found.")
endif()

if(mynteyed_FOUND)
   message(STATUS "MYNT-EYE-D-SDK found.")
   if(${mynteyed_VERSION} VERSION_LESS "1.7.2")
      message(WARNING "MYNT-EYE-D-SDK VERSION: ${mynteyed_VERSION}, camera calib info can't be identified when the sdk version is small than 1.7.2")
   endif()
else()
   message(STATUS "MYNT-EYE-D-SDK not found.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

## ###################################
## VINS-LIB
## ###################################
add_library(vins_lib
    ## estimator 
    src/estimator/get_calib_info.cpp
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    
    ## factor
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp

    ## utility 
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp

    ## initial
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    
    src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

## ###################################
## VINS-node  vins-fusion VIO
## ###################################
add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib) 


add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
target_link_libraries(kitti_odom_test vins_lib) 

add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
target_link_libraries(kitti_gps_test vins_lib) 

## ###################################
## 针对于小觅相机标准版的相机
## ###################################
if(mynteye_FOUND)
   add_library(vins_lib_s
      src/estimator/mynt_eye_ros_wrapper/get_calib_info.cpp
      src/estimator/parameters.cpp
      src/estimator/estimator.cpp
      src/estimator/feature_manager.cpp
      src/factor/pose_local_parameterization.cpp
      src/factor/projectionTwoFrameOneCamFactor.cpp
      src/factor/projectionTwoFrameTwoCamFactor.cpp
      src/factor/projectionOneFrameTwoCamFactor.cpp
      src/factor/marginalization_factor.cpp
      src/utility/utility.cpp
      src/utility/visualization.cpp
      src/utility/CameraPoseVisualization.cpp
      src/initial/solve_5pts.cpp
      src/initial/initial_aligment.cpp
      src/initial/initial_sfm.cpp
      src/initial/initial_ex_rotation.cpp
      src/featureTracker/feature_tracker.cpp)
   target_link_libraries(vins_lib_s ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} mynteye)


   add_executable(vins_s_node src/rosNodeTest.cpp)
   target_link_libraries(vins_s_node vins_lib_s) 

   add_executable(kitti_odom_test_s src/KITTIOdomTest.cpp)
   target_link_libraries(kitti_odom_test_s vins_lib_s) 

   add_executable(kitti_gps_test_s src/KITTIGPSTest.cpp)
   target_link_libraries(kitti_gps_test_s vins_lib_s) 

endif()


## ###################################
## 针对于小觅相机深度版相机  vins_lib_d
## ###################################

if(mynteyed_FOUND)
   add_library(vins_lib_d
      src/estimator/mynteye_wrapper_d/get_calib_info.cpp
      src/estimator/parameters.cpp
      src/estimator/estimator.cpp
      src/estimator/feature_manager.cpp

      src/factor/pose_local_parameterization.cpp
      src/factor/projectionTwoFrameOneCamFactor.cpp
      src/factor/projectionTwoFrameTwoCamFactor.cpp
      src/factor/projectionOneFrameTwoCamFactor.cpp
      src/factor/marginalization_factor.cpp

      src/utility/utility.cpp
      src/utility/visualization.cpp
      src/utility/CameraPoseVisualization.cpp
      
      src/initial/solve_5pts.cpp
      src/initial/initial_aligment.cpp
      src/initial/initial_sfm.cpp
      src/initial/initial_ex_rotation.cpp
      src/featureTracker/feature_tracker.cpp)
   target_link_libraries(vins_lib_d ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} mynteye_depth)


   add_executable(vins_d_node src/rosNodeTest.cpp)
   target_link_libraries(vins_d_node vins_lib_d) 

   add_executable(kitti_odom_test_d src/KITTIOdomTest.cpp)
   target_link_libraries(kitti_odom_test_d vins_lib_d) 

   add_executable(kitti_gps_test_d src/KITTIGPSTest.cpp)
   target_link_libraries(kitti_gps_test_d vins_lib_d) 

endif()

