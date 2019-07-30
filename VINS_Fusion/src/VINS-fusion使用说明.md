
# VINS-fusion使用说明  


## INPUT 数据集与传感器

- 起点的世界坐标系      `x轴朝前，y轴朝左，z轴朝上`
- Euroc数据采用双目+IMU `双目 z轴朝前，x轴朝右  // 陀螺仪 z轴朝前，x轴朝右`
- 小觅相机             `双目 z轴朝前，x轴朝右  // 陀螺仪 x轴朝右 y轴朝上，z轴朝后`
- kitti               `双目 z轴朝前，x轴朝右  // 陀螺仪 x轴朝前 y轴朝左，z轴朝上`

## OUTPUT 保存的VIO格式

- 输出路径在config中指定文件夹
- 

## loop-fusion 
目的： 后端优化为了保证VIO的实时性，主要专注于 LocalVIO,随着时间的推移，不可避免地会有漂移。
所以 loop-fusion  就是为了纠正漂移，获得更好的全局一致性。

### loop-fusion

**relocalization process**

- loop detection module 
>主要任务是当前的Keyframe 与之前所以的Keyfame比较，如果发现是之前到达的位置
就把之前的那个关键帧作为一个loop closure frame 通过 loop edge连接，加入到当前窗口的pose graph

loop detection的步骤：
1. 订阅image pose point话题
2. 选取关键帧，对于每一个关键帧提取 500个FAST feature， 并且求BRIEF描述子
3. 回环检测  找到回环帧，
4. 找到the same feature of loop closure frame and current windows featurn



pose_graph_node.cpp 
**measurement_process = std::thread(process)**
- process
- posegraph.addKeyFrame(keyframe, 1);
- loop_index = detectLoop(cur_kf, cur_kf->index);

pose_graph.cpp
**t_optimization = std::thread(&PoseGraph::optimize4DoF, this)**


## 测试数据集与仿真测试

## 代码结构

- camera_models 多传感器融合时的在校校准
((VINS-fusion)Online Temporal Calibration for Monocular Visual-Inertial Systems)

- loop_fusion

- global_fusion

- vins_estimator


## 修改


    //将body坐标系和camera坐标系旋转一下 ----
    Eigen::AngleAxisd body_camera_rotation (M_PI, Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d  boda_camera_matrax = Eigen::Matrix3d::Identity();
    boda_camera_matrax = body_camera_rotation.matrix() * estimator.ric[0];

    q.setW(Quaterniond(boda_camera_matrax).w());
    q.setX(Quaterniond(boda_camera_matrax).x());
    q.setY(Quaterniond(boda_camera_matrax).y());
    q.setZ(Quaterniond(boda_camera_matrax).z());
    //将body坐标系和camera坐标系旋转一下 ----

这样保证body坐标系与世界坐标系对齐，然后相机坐标系的Z轴指向相机前方


-在xisualization.cpp 中 pubOdometry(), 原本发布的path 时相机坐标系下的，z轴超前，
将其绕x轴旋转-90度，变成z轴朝上，世界坐标系
影响了/vins_node/path话题


## 代码结构

vins-estimator 

add vins_lib  vins_node


订阅输入所需的话题
- img0_callback  img1_callback
- imu_callback
- feature_callback
- restart_callback

实例化一个 Estimator estimator，将所需的数据都送入进去
- estimator.inputImage(time, image0, image1) or  estimator.inputImage(time, image);
- estimator.inputIMU(t, acc, gyr);
- estimator.inputFeature(t, featureFrame);

读取配置参数文件
- estimator.setParameter();



## 多线程

- std::thread sync_thread{sync_process}; 不断往 estimator送入 数据和 图片
- processThread   = std::thread(&Estimator::processMeasurements, this);
    初始化 initFirstIMUPose ==> Rs[0]
    processIMU 预积分 Rs Ps Vs Bas[] Bgs[]
    processImage 各种情况需要考虑



## BA优化
[the goal of bundle adjustment is to find 3D point positions and camera parameters that minimize the reprojection error.]()

> This optimization problem is usually formulated as a non-linear least squares problem, where the error is the squared L2 norm of the difference between the observed feature location and the projection of the corresponding 3D point on the image plane of the camera.

ceres::SizedCostFunction::Evaluate
提供输入的参数， 输出 residuals  jacobians