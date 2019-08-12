#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
  FeatureTracker();

  void readImage(const cv::Mat &_img, double _cur_time);

  void setMask();

  void addPoints();

  bool updateID(unsigned int i);

  void readIntrinsicParameter(const string &calib_file);

  void showUndistortion(const string &name);

  void rejectWithF();

  void undistortedPoints();

  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img, forw_img;
  vector<cv::Point2f> n_pts;
  vector<cv::Point2f> prev_pts, cur_pts, forw_pts; // 维护的特征点　前一帧，当前帧，以及新来的帧
  vector<cv::Point2f> prev_un_pts, cur_un_pts;     //通过相机模型投影在归一化相机平面的坐标　x/z y/z
  vector<cv::Point2f> pts_velocity;//根据连续两帧cam frame对应的特征点　cur_un_pts　prev_un_pts
  vector<int> ids;                           //每一个特征点的id
  vector<int> track_cnt;                 //每个特征点被连续追踪的次数
  map<int, cv::Point2f> cur_un_pts_map;  //current frame每一个特征点id以及　un_pts
  map<int, cv::Point2f> prev_un_pts_map; //preveal frame每一个特征点id以及　un_pts
  camodocal::CameraPtr m_camera;
  double cur_time;
  double prev_time;

  static int n_id;
};
