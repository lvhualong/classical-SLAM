

## ORB-SLAM2 ROS 使用说明

### ros_stereo
- 订阅 `/camera/left/image_raw`  `/camera/right/image_raw `左右图
- 需要参数 ORB-voc以及 双目的内参

### ros_rgbd
- 订阅 `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`



使用ros命令行 直接重定义
/cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw

要么在roslaunch 重定义话题指向
<remap from="/camera/rgb/image_color" to="/camera/rgb/image_raw" /> 