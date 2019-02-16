# classical-SLAM
一些经典的SLAM算法学习并注释的版本

## Google cartographer
 
> Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.  
- [cartographer](https://github.com/googlecartographer/cartographer)  
- [cartographer-ROS](https://github.com/googlecartographer/cartographer_ros)  
- [cartographer-README](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)
![cartographer框架](https://raw.githubusercontent.com/googlecartographer/cartographer/master/docs/source/high_level_system_overview.png)



## ORB-SLAM 框架

- ORB-SLAM

- ORB-SLAM2
> [ORB-SLAM2代码](https://github.com/raulmur/ORB_SLAM2)  
>特点： tracking, local mapping, loop closing 三线程  

- ORB-SLAM2with_pointcloud_map
> [ORB-SLAM2with_pointcloud_map代码](https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map)  
>特点： 在tracking, local mapping, loop closing 三线程的基础上,增加了semantic segmentation, dense mapping threads.  



- DS-SLAM(A Semantic Visual SLAM towards Dynamic Environments)  
>[论文](https://arxiv.org/abs/1809.08379v1) 
[代码](https://github.com/ivipsourcecode/DS-SLAM)  
>特点：在ORB-SLAM2with_pointcloud_map的基础上，增加了SegNet(pixel-wise semantic segmentation based on caffe in real-time)


## VINS 框架

- VINS-Fusion  
> 
[VINS-Fusion代码](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
VINS-Fusion is an optimization-based multi-sensor state estimator, which achieves accurate self-localization for autonomous applications (drones, cars, and AR/VR). VINS-Fusion is an extension of VINS-Mono, which supports multiple visual-inertial sensor types (mono camera + IMU, stereo cameras + IMU, even stereo cameras only). We also show a toy example of fusing VINS with GPS


## MSCKF 框架

-  [MSCKF_VIO  EKF-based VIO approach](https://github.com/KumarRobotics/msckf_vio/blob/master/README.md)  
>  The MSCKF_VIO package is a stereo version of MSCKF. The software takes in synchronized stereo images and IMU messages and generates real-time 6DOF pose estimation of the IMU frame.

- [SR-ISWF  -an extension of MSCKF]
