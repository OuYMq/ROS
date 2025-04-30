// === cloud_preprocess.cpp ===
#include "cloud_preprocess.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace faster_lio {

CloudPreprocessParams preprocess_params;

void CloudPreprocess(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_in_out) {
    if (!preprocess_params.enable_filter) return;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
    sor.setMeanK(preprocess_params.sor_mean_k);
    sor.setStddevMulThresh(preprocess_params.sor_stddev_thresh);
    sor.setInputCloud(cloud_in_out);
    sor.filter(*cloud_in_out);

    pcl::VoxelGrid<pcl::PointXYZINormal> voxel;
    voxel.setLeafSize(preprocess_params.voxel_leaf_size, preprocess_params.voxel_leaf_size, preprocess_params.voxel_leaf_size);
    voxel.setInputCloud(cloud_in_out);
    voxel.filter(*cloud_in_out);

    if (preprocess_params.remove_ground) {
        pcl::SACSegmentation<pcl::PointXYZINormal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(cloud_in_out);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);

        if (!inliers->indices.empty()) {
            pcl::ExtractIndices<pcl::PointXYZINormal> extract;
            extract.setInputCloud(cloud_in_out);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud_in_out);
        }
    }
}

} // namespace faster_lio
