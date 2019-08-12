
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sConfig_path = "../config/";
string sConfig_file = "../config/simulate_config.yaml";
string sData_path = "/home/ubuntu/coding/VIO-SLAM-course/projects/07_vins/vins_sys_code/data/";
string feature_file = "/keyframe/"; //文件夹下有每一帧cam提取的pose
string imu_data = "imu_pose.txt";   // imu_raw data
// string imu_data = "imu_pose_noise.txt";

std::shared_ptr<System> pSystem;

//将IMU数据载入system
void PubImuData()
{
	string sImu_data_file = sData_path + imu_data;
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector4d q;
	Vector3d t;
	Vector3d vAcc;
	Vector3d vGyr;

	double lastTime;
	Vector3d imuPwb;
	Quaterniond imuQwb;
	Vector3d imuVwb;
	Vector3d imuBa;
	Vector3d imuBg;
	Vector3d imuAcc;
	Vector3d imuGyro;

	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data　every line
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec 
				  >> q.w() >> q.x() >> q.y() >> q.z() 
				  >> t.x() >> t.y() >> t.z() 
				  >> vGyr.x() >> vGyr.y() >> vGyr.z() 
				  >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc); //带时间戳的IMU数据载入系统，

		//pSystem->real_poses.push_back(t);//可视化imu　real pose
		//测试imu　积分
		// pSystem->midPointIntegration(dStampNSec - lastTime,
		// 							 imuAcc, imuGyro,
		// 							 vAcc, vGyr,
		// 							 imuBa, imuBg,
		// 							 imuPwb, imuQwb, imuVwb);

		// pSystem->eulerIntegration(dStampNSec - lastTime,
		// 						  vAcc, vGyr,
		// 						  imuBa, imuBg,
		// 						  imuPwb, imuQwb, imuVwb);
		// pSystem->imu_integration_poses.push_back(imuPwb);//可视化imu积分pos

		lastTime = dStampNSec;
		imuAcc = vAcc;
		imuGyro = vGyr;

		usleep(4500 * nDelayTimes);
	}
	fsImu.close();
}

void PubRealData()
{
	string sRealData_file = sData_path +  "cam_pose.txt";
	ifstream fsPose;
	fsPose.open(sRealData_file.c_str());
	if(!fsPose.is_open())
	{
		cerr << "Failed to open realPose file! " << sRealData_file << endl;
		return;
	}
	std::string sPose_line;
	double temp;
	Vector3d current_pose;
	while (std::getline(fsPose,sPose_line)&& !sPose_line.empty())
	{
		std::istringstream ssPoseData(sPose_line);
		ssPoseData  >> temp 
					>> temp >> temp >> temp >> temp
					>> current_pose.x() >> current_pose.y() >> current_pose.z()
					>> temp >> temp >> temp
					>> temp >> temp >> temp;

		pSystem->real_poses.push_back(current_pose);
		usleep(50000 * nDelayTimes);
	}
	

}

//将image载入system
void PubImageData()
{
	string sImage_file = sData_path + "feature_files_list.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str()); //所有图像特征的文件名列表　600个
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	while (std::getline(fsImage, sImage_line) && !sImage_line.empty()) // 读取每一行，每一行都是一副图片所提的所有特征点
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;

		string imagePath = sData_path + sImgFileName;
		//读取每个camera提取的特征点

		ifstream featuresImage;
		featuresImage.open(imagePath.c_str());
		if (!featuresImage.is_open())
		{
			cerr << "Failed to open features file! " << imagePath << endl;
			return;
		}
		std::string featuresImage_line;
		std::vector<int> feature_id;
		int ids = 0;
		std::vector<Vector3d> landmark;
		std::vector<Vector2d> featurePoint;
		std::vector<Vector2d> observation_feature;
		std::vector<Vector2d> featureVelocity;
		static double lastTime;
		static std::vector<Vector2d> lastfeaturePoint(50);
		while (std::getline(featuresImage, featuresImage_line) && !featuresImage_line.empty()) // 读取一副图像的所有体征的，每一行就是一个特征点
		{
			Vector3d current_landmark;			  //真实的３D点坐标，SLAM开始是不知道这个值的，所以先暂时不用
			Vector2d current_featurePoint;		  //归一化相机坐标
			Vector3d current_observation_feature; //像素坐标
			Vector2d current_featureVelocity;	 //归一化相机坐标下点的运动速度

			Eigen::Matrix3d K;
			K << 460.0, 0, 320,
				0, 460.0, 320,
				0, 0, 0;

			double temp;
			std::istringstream ssfeatureData(featuresImage_line);
			ssfeatureData >> current_landmark.x() >> current_landmark.y() >> current_landmark.z() >> temp >> current_featurePoint.x() >> current_featurePoint.y();
			landmark.push_back(current_landmark);
			featurePoint.push_back(current_featurePoint);
			feature_id.push_back(ids);

			current_featureVelocity.x() = (current_featurePoint.x() - lastfeaturePoint[ids].x()) / (dStampNSec - lastTime);
			current_featureVelocity.y() = (current_featurePoint.y() - lastfeaturePoint[ids].y()) / (dStampNSec - lastTime);
			featureVelocity.push_back(current_featureVelocity);

			current_observation_feature = Vector3d(current_featurePoint.x(), current_featurePoint.y(), 1);
			current_observation_feature = K * current_observation_feature;

			observation_feature.push_back(Vector2d(current_observation_feature.x(), current_observation_feature.y()));


			ids++;
		}
		featuresImage.close();
		lastTime = dStampNSec;
		lastfeaturePoint = featurePoint;
		pSystem->PubFeatureData(dStampNSec, feature_id, featurePoint, observation_feature, featureVelocity); //带时间戳的feature point数据载入系统，
		usleep(50000 * nDelayTimes);
	}
}

int main(int argc, char **argv)
{
	pSystem.reset(new System(sConfig_file));

	//启动多线程
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

	sleep(1);
	std::thread thd_PubImuData(PubImuData); //imu数据的预处理－＞imu buf
	std::thread thd_PubImageData(PubImageData); //image数据预处理-> image buf

	std::thread thd_PubRealData(PubRealData); //imu数据的预处理－＞imu buf
	std::thread thd_Draw(&System::Draw, pSystem); //轨迹实时可视化的线程

	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
