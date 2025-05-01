#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PoseGraphNode {
    int id;
    double time;
    Eigen::Isometry3d pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    PoseGraphNode() : cloud(new pcl::PointCloud<pcl::PointXYZI>()) {}
};


