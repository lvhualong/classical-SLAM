#pragma once

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include <fstream>
#include <condition_variable>

// #include <cv.h>
// #include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "feature_tracker.h"

//imu for vio
struct IMU_MSG
{
    double header;                       //时间戳
    Eigen::Vector3d linear_acceleration; //数据
    Eigen::Vector3d angular_velocity;
};

typedef std::shared_ptr<IMU_MSG const> ImuConstPtr;

//image for vio
struct IMG_MSG
{
    double header;            //时间戳
    vector<Vector3d> points;  //特征点在归一化相机坐标位置　x/z y/z 1
    vector<int> id_of_point;  //point_id
    vector<float> u_of_point; //point_uv  特征点像素坐标
    vector<float> v_of_point;
    vector<float> velocity_x_of_point; //point_vx vy
    vector<float> velocity_y_of_point; //(x/z y/z) 运动速度
};

typedef std::shared_ptr<IMG_MSG const> ImgConstPtr;

class System
{
public:
    System(std::string sConfig_files);

    ~System();

    void PubImageData(double dStampSec, cv::Mat &img);

    void PubImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                    const Eigen::Vector3d &vAcc);
    void PubFeatureData(double dStampSec, const vector<int> &feature_id, const vector<Vector2d> &feature, const vector<Vector2d> &observation, std::vector<Vector2d> &featureVelocity);

    // thread: visual-inertial odometry
    void ProcessBackEnd();
    void Draw();

    void midPointIntegration(double _dt,
                             Eigen::Vector3d &acc_0, Eigen::Vector3d &gyro_0,
                             Eigen::Vector3d &acc_1, Eigen::Vector3d &gyro_1,
                             Eigen::Vector3d &acc_bias, Eigen::Vector3d &gyro_bias,
                             Eigen::Vector3d &delta_p, Eigen::Quaterniond &delta_q, Eigen::Vector3d &delta_v);

    void eulerIntegration(double _dt,
                          const Eigen::Vector3d &acc_k, const Eigen::Vector3d &gyro_k,                    // 第k帧 IMU data
                          const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias,              // IMU 偏置项，这里假定为常数
                          Eigen::Vector3d &delta_p, Eigen::Quaterniond &delta_q, Eigen::Vector3d &delta_v //前一帧result,以及updated当前帧积分result
    );

private:
    //feature tracker
    std::vector<uchar> r_status;
    std::vector<float> r_err;
    // std::queue<ImageConstPtr> img_buf;

    // ros::Publisher pub_img, pub_match;
    // ros::Publisher pub_restart;

    FeatureTracker trackerData[NUM_OF_CAM];
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    //estimator
    Estimator estimator;

    std::condition_variable con;
    double current_time = -1;
    std::queue<ImuConstPtr> imu_buf;     //IMU date buffer
    std::queue<ImgConstPtr> feature_buf; // image feature data buffer
    // std::queue<PointCloudConstPtr> relo_buf;
    int sum_of_wait = 0;

    std::mutex m_buf; // m_buf　保护imu_buf  feature_buf
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator; //保护　vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements; //一组imu和一帧图像对应的数据

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = 0;
    std::ofstream ofs_pose;
    std::vector<Eigen::Vector3d> vPath_to_draw;
    bool bStart_backend;
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();

    

public:
    vector<Vector3d> real_poses;
    vector<Vector3d> imu_integration_poses;
};