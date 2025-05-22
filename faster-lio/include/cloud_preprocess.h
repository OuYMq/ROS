// === cloud_preprocess.h ===
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>

namespace faster_lio {

struct CloudPreprocessParams {
    bool enable_filter = true;  // 一键开关滤波器
    int sor_mean_k = 50;
    double sor_stddev_thresh = 1.0;
    float voxel_leaf_size = 0.2f;
    bool remove_ground = false;
};

extern CloudPreprocessParams preprocess_params;

void CloudPreprocess(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_in_out);

} // namespace faster_lio
