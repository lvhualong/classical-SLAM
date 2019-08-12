
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
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/ubuntu/dataset/EuRoc/MH-05/";
string sConfig_path = "../config/";
string sConfig_file = "../config/euroc_config.yaml";

std::shared_ptr<System> pSystem;

//将IMU数据载入system
void PubImuData()
{
	string sImu_data_file = sConfig_path + "MH_05_imu0.txt";
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
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data　every line
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);//带时间戳的IMU数据载入系统，
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

//将image载入system
void PubImageData()
{
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());//图片文件名list
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file list file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;
	
	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())//对于每一副图像
	{
		std::istringstream ssImageData(sImage_line);
		ssImageData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);//读取每一帧图片
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		pSystem->PubImageData(dStampNSec / 1e9, img);//带时间戳的图片数据载入系统，执行track一整套流程
		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(50000*nDelayTimes);//注意这个时间是imu时间的10倍
	}
	fsImage.close();
}


int main(int argc, char **argv)
{
	// if(argc != 3)
	// {
	// 	cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
	// 		<< "For example: ./run_euroc /home/ubuntu/dataset/EuRoc/MH-05/ ../config/"<< endl;
	// 	return -1;
	// }
	// sData_path = argv[1];
	// sConfig_path = argv[2];

	pSystem.reset(new System(sConfig_file));
	
	//启动多线程
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData);//imu数据的预处理－＞imu buf

	std::thread thd_PubImageData(PubImageData);//image数据预处理-> image buf
	
	//std::thread thd_Draw(&System::Draw, pSystem);//轨迹实时可视化的线程
	
	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
