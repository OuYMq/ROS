#include "cloud_filter.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

namespace faster_lio {

pcl::PointCloud<pcl::PointXYZI>::Ptr
CloudFilter::Filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud)
{
    if (!input_cloud || input_cloud->empty()) {
        return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    size_t original_points = input_cloud->size();

    // VoxelGrid 下采样
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(input_cloud);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter(*voxel_filtered);

    size_t voxel_points = voxel_filtered->size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud = voxel_filtered;

    // 可选去噪
    if (enable_denoise_) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(voxel_filtered);
        sor.setMeanK(30);
        sor.setStddevMulThresh(1.0);
        pcl::PointCloud<pcl::PointXYZI>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZI>);
        sor.filter(*denoised);
        final_cloud = denoised;
    }

    size_t final_points = final_cloud->size();

    // 日志
    if (enable_log_) {
        std::cout << "[CloudFilter] 原始点数: " << original_points
                  << " 下采样后: " << voxel_points
                  << " 最终输出: " << final_points << std::endl;
    }

    return final_cloud;
}

} // namespace faster_lio


