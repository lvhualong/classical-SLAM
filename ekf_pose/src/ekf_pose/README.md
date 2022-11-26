# pose_ekf
Extented Kalman Filter for 6D pose estimation using gps, imu, magnetometer and sonar sensor.

#state for kalman filter
0-3 quaternion

4-6 Px Py Pz

7-9 Vx Vy Vz

10-12 bwx bwy bwz

13-15 bax bay baz 

#inertial frame: ENU

# How to run the code
    cd catkin_ws/src
    git clone git@github.com:libing64/pose_ekf.git
    cd ..
    catkin_make -DCATKIN_WHITELIST_PACKAGES="pose_ekf"
    roslaunch pose_ekf pose_ekf.launch
#rosbag for test
1. download the rosbag

    https://drive.google.com/folderview?id=0B4hFvojO5r3scWJRVWdhSmdLd0k&usp=sharing
    
2. replay the rosbag

    rosbag play 2016-03-09-22-11-07.bag

![pose_ekf1](https://cloud.githubusercontent.com/assets/3192355/13659245/5f6d9e70-e6ba-11e5-8baa-edfb05460506.png)


![pose_ekf2](https://cloud.githubusercontent.com/assets/3192355/13659246/5f6e8b1e-e6ba-11e5-8cb1-f212e1b0b8dc.png)

# reference:

http://web.cs.iastate.edu/~cs577/handouts/quaternion.pdf