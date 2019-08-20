
## 安装过程

sudo apt-get install -y python-wstool python-rosdep ninja-build

### 本地安装protobuf依赖

        VERSION="v3.4.1"
        git clone https://github.com/google/protobuf.git
        cd protobuf
        git checkout tags/${VERSION}
        mkdir build
        cd build
        cmake -G Ninja \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -Dprotobuf_BUILD_TESTS=OFF \
        -DCMAKE_INSTALL_PREFIX=../install \
        ../cmake
        ninja
        ninja install


### 本地编译cartographer

        mkdir build
        cd build
        cmake -G Ninja \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -Dprotobuf_BUILD_TESTS=OFF \
        -DCMAKE_PREFIX_PATH="${PWD}/../../protobuf/install;${CMAKE_PREFIX_PATH}" \
        -DCMAKE_INSTALL_PREFIX=../install \
        ..
        ninja
        ninja install



### 编译cartographer_ros workspace

         注意修改 cartographer库的路径
          在/cartographer_ros_ws/src/cartographer_ros-1.0.0/cartographer_ros/CMakeLists.txt　中

```
        # set(cartographer_DIR  /home/ubuntu/coding/classical-SLAM/cartographer/cartographer-1.0.0)
        find_package(cartographer REQUIRED
                PATHS /home/ubuntu/coding/classical-SLAM/cartographer/cartographer-1.0.0/install)
        message("my cartographer path : ${cartographer_DIR}")


catkin_make

```

## Test Demo

**maping**
roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/dataset/lidar_bag/2d_demo_deutsches_museum.bag


**pbstream 转　ros_map**
rosrun cartographer_ros cartographer_pbstream_to_ros_map  -map_filestem=map -pbstream_filename=2d_demo_deutsches_museum.bag.pbstream  -resolution=0.05

**localization**
roslaunch cartographer_ros demo_backpack_2d_localization.launch  \
load_state_filename:=${HOME}/dataset/lidar_bag/2d_demo_deutsches_museum.pbstream  \
bag_filename:=${HOME}/dataset/lidar_bag/2d_demo_deutsches_museum.bag

**validate bag**
rosrun cartographer_ros cartographer_rosbag_validate   -bag_filename ${HOME}/dataset/lidar_bag/2d_demo_deutsches_museum.bag


----------------------------------------------------------------------------------------------


## Run your own robot or bag

**config URDF　file**

**config  lua file**

**launch a launch file**


```
        roslaunch cartographer_ros robot_husky_mapping.launch
        roslaunch husky_robot_description gazebo_husky_robot.launch

        //键盘控制机器人移动
        roslaunch mrobot_teleop mrobot_teleop.launch

        // save the map
        rosservice call /finish_trajectory "trajectory_id: 0" 
       rosservice call  /write_state "/home/ubuntu/test.pbstream"

        // 使用刚才走一圈建好的map 进行重定位
        roslaunch cartographer_ros robot_husky_localization_navigation.launch

        //导航
        roslaunch husky_robot_description gazebo_husky_robot.launch
        roslaunch cartographer_ros robot_husky_localization_navigation.launch

        roslaunch aibee_robot move_base_global_local_noly.launch


        roslaunch cartographer_ros online_vr_bot_localization2.launch load_state_filename:=/home/aibee/ros-navigation_workspace/map_data/office_2/office.pbstream


    position: 
      x: -0.114598155022
      y: -2.17928552628
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.0882152454545
      w: 0.996101435833


  pose: 
    position: 
      x: -0.0363106429577
      y: -2.31331682205
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.0775713472395
      w: 0.996986803367



```
注意：　
在启动人仿真之后，应该首先有一个odom-->base_link的tf，以及base_link下面挂载的设备

启动cartographer_ros之后，提供一个map到odom的tf



## 主要的node文件


**cartographer_ros**
<node name="cartographer_node" pkg="cartographer_ros"  type="cartographer_node"
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename backpack_2d.lua"
                output="screen">
    <remap from="echoes" to="horizontal_laser_2d" />
  </node>


**pub /map**
<node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />