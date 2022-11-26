#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/Altimeter.h>


#include <Eigen/Geometry>
#include <deque>

#include "pose_ekf.h"
#include <algorithm>

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/LocalCartesian.hpp>

std::shared_ptr<GeographicLib::Geoid> geoid;

using namespace std;
using namespace Eigen;

enum SensorType
{
  IMU,
  FIX,
  FIX_VELOCITY,
  MAGNETIC,
  SONAR,
  ALTIMETER,
};


Pose_ekf pose_ekf;
ros::Publisher pose_pub;

nav_msgs::Path path_msg;
ros::Publisher pub_path;

deque< pair<double, sensor_msgs::Imu> > imu_q;
deque< pair<double, geometry_msgs::Vector3Stamped> > mag_q;
deque< pair<double, hector_uav_msgs::Altimeter> >altimeter_q;
deque< pair<double, sensor_msgs::Range> >sonar_height_q;
deque< pair<double, Vector3d> >fix_q;
deque< pair<double, geometry_msgs::Vector3Stamped> >fix_velocity_q;

void publish_pose(Pose_ekf pose_ekf)
{
  geometry_msgs::PoseStamped pose;
  Quaterniond q;
  Vector3d p, v, bw, ba;
  pose_ekf.getState(q, p, v, bw, ba);
  pose.header.stamp = ros::Time(pose_ekf.get_time());
  pose.header.frame_id = "/world";
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.position.x = p(0);
  pose.pose.position.y = p(1);
  pose.pose.position.z = p(2);

  pose_pub.publish(pose);
}


bool loadModels() 
{
  
  if (!geoid) {
    try {
      geoid = std::make_shared<GeographicLib::Geoid>("egm84-15", "/home/ubuntu/coding/classical-SLAM/ekf_pose/src/src/geoids");
    }
    catch (GeographicLib::GeographicErr &e) {
      ROS_ERROR("Failed to load geoid. Reason: %s", e.what());
      return false;
    }
  }
  
  return true;
}


bool processSensorData()
{
  
  if(imu_q.empty() || (imu_q.back().first - imu_q.front().first) < 0.15 ) return false;

  static int imu_cnt = 0;//correct with acc every 10 times
  //find the first com sensor
  double t[6] = {DBL_MAX};
  for(int i = 0; i < 6; i++) t[i] = DBL_MAX;
  if(!imu_q.empty()) t[0] = imu_q.front().first;
  if(!mag_q.empty()) t[1] = mag_q.front().first;
  if(!altimeter_q.empty()) t[2] = altimeter_q.front().first;
  if(!sonar_height_q.empty()) t[3] = sonar_height_q.front().first;
  if(!fix_q.empty()) t[4] = fix_q.front().first;
  if(!fix_velocity_q.empty()) t[5] = fix_velocity_q.front().first;
  
  //for(int i = 0; i < 6; i++) cout << i << " " << t[i] << "  ";
  int min_id = min_element(t, t + 6) - t;
  //cout << "min_id: " << min_id << "  min_t: " << t[min_id] << endl;
  if(t[min_id] == DBL_MAX) return false;

  
  if(min_id == 0)//imu
  {
        
        //cout << "size: " << imu_q.size() << endl;
        double t = imu_q.front().first;
        sensor_msgs::Imu msg = imu_q.front().second;
        Vector3d acc, gyro;
        acc(0) = msg.linear_acceleration.x;
        acc(1) = msg.linear_acceleration.y;
        acc(2) = msg.linear_acceleration.z;
        gyro(0) = msg.angular_velocity.x;
        gyro(1) = msg.angular_velocity.y;
        gyro(2) = msg.angular_velocity.z;
        pose_ekf.predict(gyro, acc, t);
        imu_cnt++;
        if(imu_cnt % 10 == 0) pose_ekf.correct_gravity(acc, t);
        imu_q.pop_front();
        
    } else if(min_id == 1)//magnetic 
    {
      double t = mag_q.front().first;
      geometry_msgs::Vector3Stamped msg = mag_q.front().second;
      Vector3d mag;
      mag(0) = msg.vector.x;
      mag(1) = msg.vector.y;
      mag(2) = msg.vector.z;
      pose_ekf.correct_magnetic_field(mag, t);
      mag_q.pop_front();
    } else if(min_id == 2)//altimeter
    {

    }else if(min_id == 3) //sonar height
    {
      
      double t = sonar_height_q.front().first;
      sensor_msgs::Range msg = sonar_height_q.front().second;
      double sonar_height = msg.range;
      pose_ekf.correct_sonar_height(sonar_height, t);
      sonar_height_q.pop_front();
      
    }else if(min_id == 4)//fix
    {
      
      double t = fix_q.front().first;
      Vector3d position = fix_q.front().second;
      pose_ekf.correct_fix(position, t);
      fix_q.pop_front();
      
    }else if(min_id == 5) //fix_velocity
    {
      
      double t = fix_velocity_q.front().first;
      geometry_msgs::Vector3Stamped msg = fix_velocity_q.front().second;
      Vector3d fix_velocity;
      fix_velocity(0) = msg.vector.x;
      fix_velocity(1) = msg.vector.y;
      fix_velocity(2) = msg.vector.z;
      pose_ekf.correct_fix_velocity(fix_velocity, t);
      fix_velocity_q.pop_front();
      
    }

 
  return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    imu_q.push_back(make_pair(t, *imu_msg) );
}

void magCallback(const geometry_msgs::Vector3StampedConstPtr &msg)
{
  double t = msg->header.stamp.toSec();
  mag_q.push_back(make_pair(t, *msg));
}

void altimeterCallback(const hector_uav_msgs::AltimeterConstPtr& msg)
{
  double t = msg->header.stamp.toSec();
  //altimeter_q.push_back(make_pair(t, *msg));
  //cout << "altimeter; ";
}

void sonarCallback(const sensor_msgs::RangeConstPtr &msg)
{
  double t = msg->header.stamp.toSec();
  sonar_height_q.push_back(make_pair(t, *msg));
  //cout << "sonar; ";
}

void fixCallback(const sensor_msgs::NavSatFixConstPtr & msg)
{
  static GeographicLib::LocalCartesian refPoint;
  static bool refInitialized = false;
  double t = msg->header.stamp.toSec();

  //convert to height above sea level, input is degree
  double hMSL = geoid->ConvertHeight(msg->altitude, msg->longitude, msg->altitude, GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  if(!refInitialized)
  {
    refPoint = GeographicLib::LocalCartesian(msg->latitude, msg->longitude, hMSL);
    refInitialized = true;
  }

  double locX, locY, locZ;
  refPoint.Forward(msg->latitude, msg->longitude, hMSL, locX, locY, locZ);
  Vector3d position;
  //todo, the data should be ENU, but it sames in NED? maybe a buf of hector quadrotor
  position(0) = locY;
  position(1) = -locX;
  position(2) = locZ;
  // position(0) = locX;
  // position(1) = locY;
  // position(2) = locZ;
  fix_q.push_back(make_pair(t, position));
}

void fixVelocityCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
    double t = msg->header.stamp.toSec();
    fix_velocity_q.push_back(make_pair(t, *msg));
}

int main (int argc, char **argv) 
{

  ros::init(argc, argv, "pose_estimator");
  ros::NodeHandle n("~");

  pub_path = n.advertise<nav_msgs::Path>("path", 10);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/est_pose", 10);

  path_msg.header.frame_id = "world";
  
  ros::Subscriber sub_imu = n.subscribe("imu", 100, imuCallback);
  ros::Subscriber sub_mag = n.subscribe("magnetic_field", 100, magCallback);
  ros::Subscriber sub_fix = n.subscribe("fix", 100, fixCallback);
  ros::Subscriber sub_sonar = n.subscribe("sonar_height", 100, sonarCallback); 
  ros::Subscriber sub_fix_velocity = n.subscribe("fix_velocity", 100, fixVelocityCallback);
  ros::Subscriber sub_altimeter = n.subscribe("altimeter", 100, altimeterCallback);
  

  bool ret = loadModels();
  if(!ret) return -1;

  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    ros::spinOnce();

    while(processSensorData()){}
    publish_pose(pose_ekf);

    loop_rate.sleep();
  }
  return 0;
}

