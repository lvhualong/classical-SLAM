cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)


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

## 生成的可执行文件在工程的 bin目录下
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

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
    ${PROJECT_SOURCE_DIR}/include
    )

include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${PROJECT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

## 编译生成可执行文件 
# rosbuild_add_executable(${PROJECT_NAME}
add_executable( ${PROJECT_NAME}
    src/main.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FramePublisher.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Map.cc
    src/MapPublisher.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
)
# rosbuild_add_boost_directories()
# rosbuild_link_boost(${PROJECT_NAME} thread)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

