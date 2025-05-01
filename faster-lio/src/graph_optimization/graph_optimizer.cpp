#include "graph_optimization/graph_optimizer.h"


GraphOptimizer::GraphOptimizer() {
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    optimizer_.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(solver)));
    optimizer_.setVerbose(false);
}

void GraphOptimizer::AddNode(const PoseGraphNode& node) {
    auto* v = new g2o::VertexSE3();
    v->setId(node.id);
    v->setEstimate(node.pose);
    if (node.id == 0) v->setFixed(true);  // 固定第一帧防止漂移
    optimizer_.addVertex(v);
    vertices_[node.id] = v;
}

void GraphOptimizer::AddEdge(int id1, int id2, const Eigen::Isometry3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information) {
    auto* e = new g2o::EdgeSE3();
    e->vertices()[0] = vertices_.at(id1);
    e->vertices()[1] = vertices_.at(id2);
    e->setMeasurement(relative_pose);
    e->setInformation(information);
    optimizer_.addEdge(e);
}

void GraphOptimizer::Optimize(int iterations) {
    optimizer_.initializeOptimization();
    optimizer_.optimize(iterations);
}

std::vector<Eigen::Isometry3d> GraphOptimizer::GetAllPoses() const {
    std::vector<Eigen::Isometry3d> poses(vertices_.size());
    for (const auto& [id, v] : vertices_) {
        poses[id] = v->estimate();
    }
    return poses;
}


