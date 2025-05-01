#include "graph_optimization/loop_detector.h"

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <limits>

LoopDetector::LoopDetector() {}

void LoopDetector::AddKeyframe(int id, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud) {
    keyframes_[id] = cloud;
}

bool LoopDetector::DetectLoop(int current_id,
                              const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& current_cloud,
                              int& matched_id,
                              Eigen::Isometry3d& relative_pose) {
    double best_score = std::numeric_limits<double>::max();
    int best_id = -1;
    Eigen::Isometry3d best_pose = Eigen::Isometry3d::Identity();

    for (const auto& [id, cloud] : keyframes_) {
        if (id >= current_id - 30) continue; // 跳过临近帧（避免和自身或相邻帧匹配）

        Eigen::Isometry3d pose;
        if (AlignCloud(current_cloud, cloud, pose)) {
            double score = (pose.translation()).norm();
            if (score < best_score) {
                best_score = score;
                best_id = id;
                best_pose = pose;
            }
        }
    }

    if (best_id != -1 && best_score < fitness_score_threshold_) {
        matched_id = best_id;
        relative_pose = best_pose;
        return true;
    }
    return false;
}

bool LoopDetector::AlignCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source,
                              const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target,
                              Eigen::Isometry3d& relative_pose) {
    pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    pcl::PointCloud<pcl::PointXYZINormal> aligned;
    icp.align(aligned);

    if (!icp.hasConverged() || icp.getFitnessScore() > fitness_score_threshold_) {
        return false;
    }

    Eigen::Matrix4f T = icp.getFinalTransformation();
    relative_pose = Eigen::Isometry3d(T.cast<double>());
    return true;
}

