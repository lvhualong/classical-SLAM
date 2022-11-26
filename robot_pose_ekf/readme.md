
# robot_pose_EKF


## 订阅的话题

- odom  轮速计　　   nav_msgs::Odometry　[with_odom_covariance_6*6]
- IMU   惯导测量     sensor_msgs::Imu [如果没有设置协方差，则手动设置方差]
- 