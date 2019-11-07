/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"
#include "utility/gps_conversion.h"

// 这个文件仅仅是把　VINS-stereo 估计的结果和GPS数据都发布成ROS topic
// 算法中并没有做　VINS-SLAM与GPS的融合
using namespace std;
using namespace Eigen;

Estimator estimator;

ros::Publisher pubGPS;
ros::Publisher pubGPS_Point;
ros::Publisher pubGPS_Pose;

GPSConversion gpsConversion_;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	pubGPS = n.advertise<sensor_msgs::NavSatFix>("/gps", 1000);
	pubGPS_Point = n.advertise<geometry_msgs::PointStamped>("/gps_point", 100);
	pubGPS_Pose = n.advertise<geometry_msgs::PoseStamped>("/gps_pose", 100);
	static bool referenceinit = false; //接收第一帧GPS参考帧数据

	if (argc != 3)
	{
		printf("please intput: rosrun vins kitti_gps_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_gps_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml "
			   "/media/tony-ws1/disk_D/kitti/2011_10_03/2011_10_03_drive_0027_sync/ \n");
		return 1;
	}

	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
	string sequence = argv[2];
	printf("read sequence: %s\n", argv[2]);
	string dataPath = sequence + "/";

	// load image list
	FILE *file;
	file = std::fopen((dataPath + "image_00/timestamps.txt").c_str(), "r");
	if (file == NULL)
	{
		printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
		ROS_BREAK();
		return 0;
	}
	vector<double> imageTimeList; //所以图片左图的时间戳
	int year, month, day;
	int hour, minute;
	double second;
	while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
	{
		//printf("%lf\n", second);
		imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
	}
	std::fclose(file);

	// load gps list
	vector<double> GPSTimeList; //所有对应的GPS的时间戳
	{
		FILE *file;
		file = std::fopen((dataPath + "oxts/timestamps.txt").c_str(), "r");
		if (file == NULL)
		{
			printf("cannot find file: %soxts/timestamps.txt \n", dataPath.c_str());
			ROS_BREAK();
			return 0;
		}
		int year, month, day;
		int hour, minute;
		double second;
		while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
		{
			//printf("%lf\n", second);
			GPSTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
		}
		std::fclose(file);
	}

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);

	FILE *outFile;
	outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(), "w");
	if (outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	double baseTime;

	for (size_t i = 0; i < imageTimeList.size(); i++)
	{
		if (ros::ok())
		{
			if (imageTimeList[0] < GPSTimeList[0])
				baseTime = imageTimeList[0];
			else
				baseTime = GPSTimeList[0];

			//printf("base time is %f\n", baseTime);
			//printf("process image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(10) << i;
			leftImagePath = dataPath + "image_00/data/" + ss.str() + ".png";
			rightImagePath = dataPath + "image_01/data/" + ss.str() + ".png";
			//printf("%s\n", leftImagePath.c_str());
			//printf("%s\n", rightImagePath.c_str());

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE);
			imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE);

			double imgTime = imageTimeList[i] - baseTime;

			// load gps
			FILE *GPSFile;
			string GPSFilePath = dataPath + "oxts/data/" + ss.str() + ".txt";
			GPSFile = std::fopen(GPSFilePath.c_str(), "r");
			if (GPSFile == NULL)
			{
				printf("cannot find file: %s\n", GPSFilePath.c_str());
				ROS_BREAK();
				return 0;
			}
			double lat, lon, alt, roll, pitch, yaw;
			double vn, ve, vf, vl, vu;
			double ax, ay, az, af, al, au;
			double wx, wy, wz, wf, wl, wu;
			double pos_accuracy, vel_accuracy;
			double navstat, numsats;
			double velmode, orimode;

			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
			//printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
			//printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
			//printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
			//printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);
			fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode, &orimode);
			//printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n",
			//	    pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);

			std::fclose(GPSFile);

			sensor_msgs::NavSatFix gps_position;
			gps_position.header.frame_id = "world"; //NED
			gps_position.header.stamp = ros::Time(imgTime);
			gps_position.status.status = navstat;
			gps_position.status.service = numsats;
			gps_position.latitude = lat;
			gps_position.longitude = lon;
			gps_position.altitude = alt;
			gps_position.position_covariance[0] = pos_accuracy;
			//printf("pos_accuracy %f \n", pos_accuracy);
			//pubGPS.publish(gps_position);

			if (!referenceinit)
			{
				gpsConversion_.InitReference(lat, lon, alt);
				ROS_WARN_STREAM("Init GPS reference");
				referenceinit = true;
			}
			Eigen::Vector3d enu_pose = gpsConversion_.WGS84ToENU(lat, lon, alt);
			Eigen::Vector3d ea0(yaw, pitch, roll);
			Eigen::Matrix3d R;
			R = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
			Eigen::Quaterniond enu_q(R);

			//把enu转到相机坐标
			Eigen::Vector3d trans(0, pitch, 0.5 * M_PI);
			Eigen::Matrix3d trans_R;
			trans_R = Eigen::AngleAxisd(trans[0], Eigen::Vector3d::UnitZ()) *
					  Eigen::AngleAxisd(trans[1], Eigen::Vector3d::UnitY()) *
					  Eigen::AngleAxisd(trans[2], Eigen::Vector3d::UnitX());

			enu_pose = trans_R*enu_pose;

			geometry_msgs::PoseStamped gps_pose;
			gps_pose.header.frame_id = "world";
			gps_pose.header.stamp = ros::Time(imgTime);
			gps_pose.pose.position.x = enu_pose[0];
			gps_pose.pose.position.y = enu_pose[1];
			gps_pose.pose.position.z = enu_pose[2];
			gps_pose.pose.orientation.x = enu_q.x();
			gps_pose.pose.orientation.y = enu_q.y();
			gps_pose.pose.orientation.z = enu_q.z();
			gps_pose.pose.orientation.w = enu_q.w();
			// pubGPS_Pose.publish(gps_pose);

			geometry_msgs::PointStamped gps_point;
			gps_point.header = gps_pose.header;
			gps_point.point.x = enu_pose[0];
			gps_point.point.y = enu_pose[1];
			gps_point.point.z = enu_pose[2];
			pubGPS_Point.publish(gps_point);

			estimator.inputImage(imgTime, imLeft, imRight);

			Eigen::Matrix<double, 4, 4> pose;
			estimator.getPoseInWorldFrame(pose);
			if (outFile != NULL)
				fprintf(outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n", pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
						pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
						pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3));

			// cv::imshow("leftImage", imLeft);
			// cv::imshow("rightImage", imRight);
			// cv::waitKey(2);
		}
		else
			break;
	}
	if (outFile != NULL)
		fclose(outFile);
	return 0;
}
