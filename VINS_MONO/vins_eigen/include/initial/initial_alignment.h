#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

#include "../factor/integration_base.h"
#include "../utility/utility.h"
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

class ImageFrame
{
public:
    ImageFrame(){};
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_points, double _t) : t{_t}, is_key_frame{false}
    {
        points = _points;
    };
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;//当前帧的观测　map中　feature_id xyz_uv_velocity
    double t;
    Matrix3d R;//cam pose
    Vector3d T;
    IntegrationBase *pre_integration; //相机pose相对应的一个IMU增量
    bool is_key_frame;
};

//视觉与ＩＭＵ对齐
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs, Vector3d &g, VectorXd &x);