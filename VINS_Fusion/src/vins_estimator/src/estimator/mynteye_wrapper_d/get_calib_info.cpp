#include <ros/ros.h>
#include "GetParams.h"
#include "../get_calib_info.h"
#define CONFIGURU_IMPLEMENTATION 1
#include "../configuru.hpp"
#include "../parameters.h"
using namespace configuru;

std::string get_calib_info(int type) {
    ros::NodeHandle ns;
    ros::ServiceClient client =
        ns.serviceClient<mynteye_wrapper_d::GetParams>("/mynteye_wrapper_d_node/get_params");
    mynteye_wrapper_d::GetParams srv;
    // IMG_INTRINSICS = 0u,
    // IMG_EXTRINSICS_RTOL = 1u,
    // IMU_INTRINSICS = 2u,
    // IMU_EXTRINSICS = 3u,
    srv.request.key = type;  // IMG_INTRINSICS
    if (client.call(srv)) {
      return srv.response.value;
    } else {
      ROS_ERROR("Failed to call service GetParams, make sure you have launch mynteye device SDK nodelet");
      return "null";
    }
}

//通过SDK 读取参数写入文件
void ConversionFromDevice(const std::string& left_path, const std::string& right_path, Config img_intri_info, Config img_extri_info) {
  std::cout << "intrinsics: \n" << img_intri_info << std::endl;
  std::cout << "extrinsics: \n" << img_extri_info << std::endl;
  cv::FileStorage calib_intri_fs_left(left_path, cv::FileStorage::WRITE);
  cv::FileStorage calib_intri_fs_right(right_path, cv::FileStorage::WRITE);

  //kannala_brandt calibration model
  if (img_intri_info["calib_model"] == "kannala_brandt") {

    calib_intri_fs_left << "model_type" << "KANNALA_BRANDT";
    calib_intri_fs_left << "camera_name" << "kannala-brandt";
    calib_intri_fs_left << "image_width" << (int)img_intri_info["left"]["width"];
    calib_intri_fs_left << "image_height" << (int)img_intri_info["left"]["height"];
    calib_intri_fs_left << "projection_parameters";
    calib_intri_fs_left << "{";
    calib_intri_fs_left << "k2" << (double)img_intri_info["left"]["coeffs"][0];
    calib_intri_fs_left << "k3" << (double)img_intri_info["left"]["coeffs"][1];
    calib_intri_fs_left << "k4" << (double)img_intri_info["left"]["coeffs"][2];
    calib_intri_fs_left << "k5" << (double)img_intri_info["left"]["coeffs"][3];
    calib_intri_fs_left << "mu" << (double)img_intri_info["left"]["coeffs"][4];
    calib_intri_fs_left << "mv" << (double)img_intri_info["left"]["coeffs"][5];
    calib_intri_fs_left << "u0" << (double)img_intri_info["left"]["coeffs"][6];
    calib_intri_fs_left << "v0" << (double)img_intri_info["left"]["coeffs"][7];
    calib_intri_fs_left << "}";

    calib_intri_fs_right << "model_type" << "KANNALA_BRANDT";
    calib_intri_fs_right << "camera_name" << "kannala-brandt";
    calib_intri_fs_right << "image_width" << (int)img_intri_info["right"]["width"];
    calib_intri_fs_right << "image_height" << (int)img_intri_info["right"]["height"];
    calib_intri_fs_right << "projection_parameters";
    calib_intri_fs_right << "{";
    calib_intri_fs_right << "k2" << (double)img_intri_info["right"]["coeffs"][0];
    calib_intri_fs_right << "k3" << (double)img_intri_info["right"]["coeffs"][1];
    calib_intri_fs_right << "k4" << (double)img_intri_info["right"]["coeffs"][2];
    calib_intri_fs_right << "k5" << (double)img_intri_info["right"]["coeffs"][3];
    calib_intri_fs_right << "mu" << (double)img_intri_info["right"]["coeffs"][4];
    calib_intri_fs_right << "mv" << (double)img_intri_info["right"]["coeffs"][5];
    calib_intri_fs_right << "u0" << (double)img_intri_info["right"]["coeffs"][6];
    calib_intri_fs_right << "v0" << (double)img_intri_info["right"]["coeffs"][7];
    calib_intri_fs_right << "}";
  } 
  //pinhole calib model
  else if (img_intri_info["calib_model"] == "pinhole") {
    calib_intri_fs_left << "model_type" << "PINHOLE";
    calib_intri_fs_left << "camera_name" << "pinhole";
    calib_intri_fs_left << "image_width" << (int)img_intri_info["left"]["width"];
    calib_intri_fs_left << "image_height" << (int)img_intri_info["left"]["height"];
    calib_intri_fs_left << "distortion_parameters";
    calib_intri_fs_left << "{";
    calib_intri_fs_left << "k1" << (double)img_intri_info["left"]["coeffs"][0];
    calib_intri_fs_left << "k2" << (double)img_intri_info["left"]["coeffs"][1];
    calib_intri_fs_left << "p1" << (double)img_intri_info["left"]["coeffs"][2];
    calib_intri_fs_left << "p2" << (double)img_intri_info["left"]["coeffs"][3];
    calib_intri_fs_left << "}";
    calib_intri_fs_left << "projection_parameters";
    calib_intri_fs_left << "{";
    calib_intri_fs_left << "fx" << (double)img_intri_info["left"]["fx"];
    calib_intri_fs_left << "fy" << (double)img_intri_info["left"]["fy"];
    calib_intri_fs_left << "cx" << (double)img_intri_info["left"]["cx"];
    calib_intri_fs_left << "cy" << (double)img_intri_info["left"]["cy"];
    calib_intri_fs_left << "}";

    calib_intri_fs_right << "model_type" << "PINHOLE";
    calib_intri_fs_right << "camera_name" << "pinhole";
    calib_intri_fs_right << "image_width" << (int)img_intri_info["right"]["width"];
    calib_intri_fs_right << "image_height" << (int)img_intri_info["right"]["height"];
    calib_intri_fs_right << "distortion_parameters";
    calib_intri_fs_right << "{";
    calib_intri_fs_right << "k1" << (double)img_intri_info["right"]["coeffs"][0];
    calib_intri_fs_right << "k2" << (double)img_intri_info["right"]["coeffs"][1];
    calib_intri_fs_right << "p1" << (double)img_intri_info["right"]["coeffs"][2];
    calib_intri_fs_right << "p2" << (double)img_intri_info["right"]["coeffs"][3];
    calib_intri_fs_right << "}";
    calib_intri_fs_right << "projection_parameters";
    calib_intri_fs_right << "{";
    calib_intri_fs_right << "fx" << (double)img_intri_info["right"]["fx"];
    calib_intri_fs_right << "fy" << (double)img_intri_info["right"]["fy"];
    calib_intri_fs_right << "cx" << (double)img_intri_info["right"]["cx"];
    calib_intri_fs_right << "cy" << (double)img_intri_info["right"]["cy"];
    calib_intri_fs_right << "}";
  }
  calib_intri_fs_right.release();
  calib_intri_fs_left.release();
}

//通过MYNTSDK API 获取相机内参 外参， 并写入parameter
bool readMYNTConfig(std::string config_file) {
    auto img_intri = get_calib_info(0);
    Config img_intri_info;
    int pn__ = config_file.find_last_of('/');
    std::string configPath__ = config_file.substr(0, pn__);
    std::string device_info_path_left = configPath__ + "/device_params_left.yaml";
    std::string device_info_path_right = configPath__ + "/device_params_right.yaml";
    
    if (img_intri != "null") {
        img_intri_info = parse_string(img_intri.c_str(), JSON, "log");
        // std::cout << "intrinsics: \n" << img_intri_info << std::endl;
        // int pn = config_file.find_last_of('/');
        // std::string configPath = config_file.substr(0, pn);
        // dump_file(configPath + "/device_img_intrinsics.json", img_intri_info, JSON);
    } else {
        ROS_WARN("check the list below:");
        ROS_WARN("1. the mynteye d device ROS nodelet not been launched");
        ROS_WARN("2. the mynteye d device SDK version may be too old");
        ROS_WARN("3. the device calib data may not correct");
        return false;
    }
    auto img_extri = get_calib_info(1);
    Config img_extri_info;
    if (img_extri != "null") {
        img_extri_info = parse_string(img_extri.c_str(), JSON, "log");
        // std::cout << "extrinsics: \n" << img_extri_info << std::endl;
        // int pn = config_file.find_last_of('/');
        // std::string configPath = config_file.substr(0, pn);
        // dump_file(configPath + "/device_img_extrinsics.json", img_extri_info, JSON);
    }

    if (img_intri != "null" &&
        img_extri != "null") {
      ConversionFromDevice(device_info_path_left, device_info_path_right, img_intri_info, img_extri_info);
    }

    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL) {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return true;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if (USE_IMU) {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    if (img_intri != "null" &&
        img_extri != "null") {
      CAM_NAMES.push_back(device_info_path_left);
    } else {
      fsSettings["cam0_calib"] >> cam0Calib;
      std::string cam0Path = configPath + "/" + cam0Calib;
      CAM_NAMES.push_back(cam0Path);
    }

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        if (img_intri != "null" &&
            img_extri != "null") {
          CAM_NAMES.push_back(device_info_path_right);
        } else {
          fsSettings["cam1_calib"] >> cam1Calib;
          std::string cam1Path = configPath + "/" + cam1Calib; 
          CAM_NAMES.push_back(cam1Path);
        }
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
    return true;
}
