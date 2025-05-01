#pragma once

#include <unordered_map>
#include <Eigen/Dense>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include "pose_graph_node.h"

class GraphOptimizer {
public:
    GraphOptimizer();

    void AddNode(const PoseGraphNode& node);
    void AddEdge(int id1, int id2, const Eigen::Isometry3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information);
    void Optimize(int iterations = 30);
    std::vector<Eigen::Isometry3d> GetAllPoses() const;

private:
    g2o::SparseOptimizer optimizer_;
    std::unordered_map<int, g2o::VertexSE3*> vertices_;
};


