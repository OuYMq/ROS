#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

class LoopDetector {
public:
    LoopDetector();

    void AddKeyframe(int id, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud);

    // 简化接口：检测是否和已有关键帧形成回环
    // 返回 true 表示检测到回环，matched_id 是回环帧的 id，relative_pose 是两帧之间的相对位姿（由 ICP 获得）
    bool DetectLoop(int current_id,
                   const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& current_cloud,
                   int& matched_id,
                   Eigen::Isometry3d& relative_pose);

private:
    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZINormal>::Ptr> keyframes_;  // 保存所有关键帧点云
    double fitness_score_threshold_ = 1.5; // ICP 匹配质量阈值（越小越严格）

    // ICP 匹配函数
    bool AlignCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source,
                    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target,
                    Eigen::Isometry3d& relative_pose);
};

