/*
*********************************
name  @function:
commented by hualong 2019/1/


*********************************
*/

## ORB-SLAM2


## 编译 ORB-SLAM2
----------------------------------------------------------------------------------
### 编译依赖库

 - DBoW2、g2o
 - 或者直接借用 build.sh脚本，编译

 ### 解压ORB voc词典

 - Vocabulary/ORBvoc.txt
 - 或者后面想办法使用二进制词典进行加速

 ### 编译C++代码
 
 - 使用 mkdir build cmake .. make 
 - 上面的build.sh也可直接编译依赖库和C++ ORB-SLAM2
 这种编译C++ 会生成一个 libORB_SLAM2.so的动态库，以及直接使用该库的一些单目、双目、RGB-D,以及直接视频流的例程

 ### 编译ROS代码

ORB-SLAM2提供的build_ros.sh采用的rosbuild的编译方式，
后面需要给改成catkin_make的方式


## 系统CmakeLists.txt文件分析
----------------------------------------------------------------------------------
### 依赖库

- cmake_modules[Eigen3 Gflag Glog]
- Thrirdparty [g2o DBoW2]
- OpenCV3
- Pangolin 
- mynteye/mynteyed SDK

### 将所有的src文件 编译成一个动态库 libORB_SLAM2.so

### 编译一些examples

- Examples/RGB-D/rgbd_tum
- Examples/Stereo/stereo_kitti
- Examples/Stereo/stereo_euroc
- Examples/Monocular/mono_tum
- Examples/Monocular/mono_kitti
- Examples/Monocular/mono_euroc

- Examples/Stereo/stereo_mynt_s

### ros CMakeLists.txt 编译了一个订阅ros图像话题的节点(位于 Examples/ROS中)
- Mono -订阅/camera/image_raw 单目图
- mynteye_mono  
- MonoAR 单目增强现实

- Stereo
- mynteye_s_stereo
- mynteye_d_stereo

- RGBD
- mynteye_rgbd



## ORB-SLAM2 使用与测试说明

**当修改ORB-SLAM2的代码后，在ORB-SLAM2的根目录 /build cmake.. make 更新ORB-SLAM2的库**

### C++ 测试

cd /home/hualong/ssd/coding/ros-SLAM-ws/src/MYNT-EYE-ORB-SLAM2-Sample

**Monocular - EuRoc 数据集**
./Examples/Monocular/mono_euroc ../ORBvoc/ORBvoc.txt Examples/Monocular/EuRoC.yaml  /home/hualong/ssd/datasets/SLAM/EuRoC/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/MH01.txt

**Monocular - KITTI-odom 数据集**
./Examples/Monocular/mono_kitti ../ORBvoc/ORBvoc.txt  Examples/Monocular/KITTI00-02.yaml /home/hualong/ssd/datasets/SLAM/KITTI-odom/data_odometry_gray/00

**Stereo -Euroc**
./Examples/Stereo/stereo_euroc  ../ORBvoc/ORBvoc.txt Examples/Stereo/EuRoC.yaml  /home/hualong/ssd/datasets/SLAM/EuRoC/mav0/cam0/data  /home/hualong/ssd/datasets/SLAM/EuRoC/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/MH01.txt

**Stereo -Kitti**
./Examples/Stereo/stereo_kitti ../ORBvoc/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml /home/hualong/ssd/datasets/SLAM/KITTI-odom/data_odometry_gray/00



### ORB-SLAM2 ROS测试
- roslaunch ORB_SLAM2 ORB_SLAM2_mono_rosbagtest.launch 使用ORB-SLAM2提供的rosbag 测试 ORB-SLAM2-mono

- roslaunch ORB_SLAM2 ORB_SLAM2_mono_MYNT-VGA.launch 使用小觅相机录制的数据集测试 单目测试 VGA分辨率



## 配置文件
- 注意图像的帧率 和 图像格式(RGB  BGR)
- 注意ORB特征的 nFeatures(特征点数)  nLevels  iniThFAST(初始fast阈值)  minThFAST(最小FAST阈值)
- 双目和RGBD ThDepth
- RGB-D DepthMapFactor