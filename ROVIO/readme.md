
# ROVOI README

## install

```bash
    ## lightweight_filtering
    git submodule update --init --recursive

    ## install kindr
    git clone https://github.com/ANYbotics/kindr.git

    ## build rovio ［放到一个workspace src下］
    catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release

    ## build with openGL [一般上面一个就可以了]
    catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
    ## 再次编译直接
    catkin build
```

## Test in the EuRoc dataset

```bash
    roslaunch rovio rovio_node.launch
    rosbag  play MH_01_easy.bag
```

## Ues in your own data

```bash

### first: write your launch file refer to rovio_node.launch
  ## remap the topic name
  <remap from="cam0/image_raw" to="/stereo/left/image_raw"/>
  <remap from="cam1/image_raw" to="/stereo/right/image_raw"/>
  <remap from="imu0" to="/mavros/imu/data_raw"/>

### second: write the config file according your sensors
  ## config the yaml file
  set image_width*height
  set camera_matrix
  set distortion_model & distortion_coefficients

  ## config the info file
  



```

## the config file

- cfg/YAML file: Camera matrix and distortion parameters  
- cfg/rovio.info: most parameters for rovio　　
  
包含相机与IMU相对外参　**qCM** (quaternion from IMU to camera frame, Hamilton-convention) and **MrMC** (Translation between IMU and Camera expressed in the IMU frame，**它们会被在线估计，但是要给一组初值**　　

##