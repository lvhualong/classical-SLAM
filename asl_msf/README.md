# ethzasl_msf

## 参数配置

> These parameters belong to the main MSF programm and not to a specific sensor and are necessary independent of the sensors in use.

```r
## MSF core
    scale_init: scaling factor １. 代表没有缩放
    core/data_playback: be set True if you are running the system from a ROSbag and False otherwise
    注意，使用ROSbag的时候，to use simulation time to true: "roscore set /use_sim_time True".
        　并且　play your ROSbag using the "--clock" option.
    core/fixed_bias：True the IMU bias can not be adapted online by MSF.　通过设置为false,除非你确信bias不会变
    core IMU parameters: core/core_noise_acc, core/core_noise_accbias, core/core_noise_gyr, core/core_noise_gyrbias

## MSF single sensor Param 每一个sensor的配置文件都是独立的
    Transformation parameters: sensor的测量与机体中心也就是IMU body的相对变换　sensortype/init/q_ic/x, sensortype/init/q_ic/y...
    sensortype/type_absolute_measurements: 确定传感器的测量是绝对测量，还是相对于上一帧的相对测量
    sensortype/type_use_fixed_covariance: True表示传感器的协方差是固定的，False表示协方差由传感器提供　(如果传感器不提供一般设为True))
    sensortype/type_measurement_world_sensor: True表示　传感器的测量位姿或者估计位姿为　Ｐwb (body to world)[如Vicon ROVIO], false 表示Ｐbw[如　ptam]]

## stability and precision
    Outlier rejection
        sensortype/enable_mah_outlier_rejection 一般设置为true，除非你十分确信你的传感器或者算法不会产生outlier
        sensortype/mah_threshold  一般设置为５，　一把根据噪声和协方差来设置

    Noise Estimation  只有没有固定协方差，才会估计噪声
        sensortype/enable_noise_estimation
        sensortype/noise_estimation_discount_factor
        sensortype/max_noise_threshold　限制所估计噪声的上限

    Sensor Recovery
        sensortype/enable_divergence_recovery:　　enables or disables sensor recovery.
        sensortype/divergence_rejection_limit
        sensortype/use_reset_to_pose defines
```

## 输入输出数据

- GPS数据

```r
latitude: 47.6416979
longitude: -122.1386583
altitude: 101.009385831
position_covariance: [0.089401, 0.0, 0.0, 0.0, 0.089401, 0.0, 0.0, 0.0, 0.159201]
position_covariance_type: 2

```

```bash
    msf_core/hl_state_input
    msf_core/correction
    msf_core/imu_state_input
    msf_updates/transform_input
```

## 程序结构

- IMUHandler_ROS

```r
## 订阅　imu测量数据　imu_state_input
    数据类型：sensor_msgs::Imu
    ProcessIMU(acc, angvel, stamp, seq);
    积分并求解IMU协方差

## 订阅　pose数据
    数据类型: geometry_msgs::PoseWithCovarianceStamped
             geometry_msgs::TransformStamped
             geometry_msgs::PoseStamped
    MeasurementCallback
    ProcessPoseMeasurement(pose);


## 订阅　hl_state_input

```

- PoseSensorHandler_T

## Description

Time delay compensated single and multi sensor fusion framework based on an EKF.
Please see the wiki for more information: https://github.com/ethz-asl/ethzasl_msf/wiki

## Documentation
The API is documented here: http://ethz-asl.github.io/ethzasl_msf

## Contributing
You are welcome contributing to the package by opening a pull-request:
Please make yourself familiar with the Google c++ style guide: 
http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml

## License
The source code is released under the Apache 2.0 license
