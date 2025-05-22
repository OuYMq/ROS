#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>

geometry_msgs::PoseWithCovarianceStamped pose_with_cov;
ros::Publisher pose_with_cov_pub;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   pose_with_cov.header = msg->header;
    pose_with_cov.pose.pose = msg->pose.pose;
    std::cout<<"pub_pose_with_cov"<<" pose"<<pose_with_cov.pose<<std::endl;
    for(int i = 0; i < 36; ++i)
    {
        pose_with_cov.pose.covariance[i] = msg->pose.covariance[i];
        // std::cout<<"pub_pose_with_cov"<<" cov"<<pose_with_cov.covariance[10]<<std::endl;
    }
    pose_with_cov_pub.publish(pose_with_cov);
     std::cout<<"pubsuccessful!"<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom2pose");
    ros::NodeHandle nh;
    pose_with_cov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris_0/mavros/vision_pose/pose_cov", 10);
    ros::Subscriber odom_sub = nh.subscribe("Odometry", 10, odomCallback);
    ros::spin();
    return 0;
}
