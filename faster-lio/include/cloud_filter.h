#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace faster_lio {

/**
 * 点云滤波模块：VoxelGrid 下采样 + 可选去噪
 */
class CloudFilter {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CloudFilter() = default;
    ~CloudFilter() = default;

    // 配置项
    void SetVoxelLeafSize(float size) { voxel_leaf_size_ = size; }
    void EnableDenoise(bool enable) { enable_denoise_ = enable; }
    void EnableLog(bool enable) { enable_log_ = enable; }

    // 滤波处理函数
    pcl::PointCloud<pcl::PointXYZI>::Ptr
    Filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud);

   private:
    float voxel_leaf_size_ = 0.2f;
    bool enable_denoise_ = false;
    bool enable_log_ = true;
};

} // namespace faster_lio


