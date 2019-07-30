
## launch mynteye camera
roslaunch mynteye_wrapper_d mynteye-vga-raw.launch

## waitting 5 seconds for camera Init
## sleep 5

## launch VINS-Fusion with Stereo+IMU
roslaunch vins mynteye-d-stereo-imu.launch

## mapping
roslaunch open_quadtree_mapping  example-mynteye.launch
