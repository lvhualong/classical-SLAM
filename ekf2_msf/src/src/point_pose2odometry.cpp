#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/Odometry.h>

ros::Publisher Odom_pub;

void point_Callback(const geometry_msgs::PointStampedConstPtr &point_data)
{
    nav_msgs::Odometry global_pose;
    std_msgs::Header header;
    header.frame_id = "world";
    header.seq = point_data->header.seq;
    header.stamp = point_data->header.stamp;

    global_pose.header = header;
    global_pose.pose.pose.position = point_data->point;

    Odom_pub.publish(global_pose);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gps_conver");
	ros::NodeHandle nh;
    ros::Subscriber pointSub = nh.subscribe<geometry_msgs::PointStamped>("global_point",1000,&point_Callback, ros::TransportHints().tcpNoDelay(true));
    Odom_pub = nh.advertise<nav_msgs::Odometry>("gps_pose", 1);
	ros::spin();
	return 0;
}