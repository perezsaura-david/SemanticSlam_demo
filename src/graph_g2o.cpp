/********************************************************************************************
 *  \file       optimizer_g2o.cpp
 *  \brief      An slam implementation for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "as2_slam/graph_g2o.hpp"

#include <g2o/core/optimization_algorithm_factory.h>

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

GraphG2O::GraphG2O(std::string _name) {
  name_ = _name;
  FLAG("Create " << name_);

  graph_                  = std::make_shared<g2o::SparseOptimizer>();  // g2o graph
  std::string solver_type = "lm_var_cholmod";                          // Check list of solver types
  INFO("construct solver: " << solver_type);
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph_->setAlgorithm(solver);

  if (!graph_->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate main solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
}

void GraphG2O::initGraph(const Eigen::Vector3d& initial_position,
                         const Eigen::Quaterniond& initial_orientation) {
  // this position will help the optimizer to find the correct solution ( its like the
  // prior )
  // Initial_pose is set to (0,0,0) by default
  Eigen::Isometry3d node_pose = Eigen::Translation3d(initial_position) * initial_orientation;
  // auto [ground, id]           = addSE3Node(Eigen::Isometry3d::Identity());
  auto [ground, id] = addSE3Node(node_pose);
  odom_nodes_.emplace_back(ground);
  ground->setFixed(true);
  last_node_      = ground;
  last_node_pose_ = node_pose;
}

void GraphG2O::optimizeGraph() {
  // g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_ptr_.get());
  const int num_iterations = 100;

  // std::cout << std::endl;
  INFO("--- pose " << name_ << " optimization ---");
  INFO("nodes: " << graph_->vertices().size() << "   edges: " << graph_->edges().size());
  // std::cout << "optimizing... " << std::flush;
  // std::cout << "init" << std::endl;
  graph_->initializeOptimization();
  graph_->setVerbose(false);

  double chi2 = graph_->chi2();
  if (std::isnan(chi2)) {
    ERROR("GRAPH RETURNED A NAN WAITING AFTER OPTIMIZATION");
  }
  // std::cout << "Start optimization" << std::endl;
  graph_->optimize(num_iterations);
  // int iterations = graph_->optimize(num_iterations);
  FLAG("Optimization done");
  // std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  // std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  if (std::isnan(graph_->chi2())) {
    throw std::invalid_argument("GRAPH RETURNED A NAN...STOPPING THE EXPERIMENT");
  }
}

std::pair<g2o::VertexSE3*, int> GraphG2O::addSE3Node(const Eigen::Isometry3d& _pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  auto id = n_vertices_++;
  vertex->setId(static_cast<int>(id));
  vertex->setEstimate(_pose);
  if (!graph_->addVertex(vertex)) {
    WARN("Vertex not added");
  }
  return {vertex, id};
}

g2o::EdgeSE3* GraphG2O::addSE3Edge(g2o::VertexSE3* _v1,
                                   g2o::VertexSE3* _v2,
                                   const Eigen::Isometry3d& _relative_pose,
                                   const Eigen::MatrixXd& _information_matrix) {
  // DEBUG("information_matrix"); // Check Information Matrix
  // DEBUG(_information_matrix);

  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setId(static_cast<int>(n_edges_++));
  edge->setMeasurement(_relative_pose);
  edge->setInformation(_information_matrix);
  edge->vertices()[0] = _v1;
  edge->vertices()[1] = _v2;
  if (!graph_->addEdge(edge)) {
    WARN("Edge not added");
  }
  return edge;
}

void GraphG2O::addNewKeyframe(const Eigen::Isometry3d& _absolute_pose,
                              const Eigen::Isometry3d& _relative_pose,
                              const Eigen::MatrixXd& _relative_covariance) {
  // std::cout << "*** NEW KEY FRAME ***" << std::endl;
  auto [node, id] = addSE3Node(_absolute_pose);
  odom_nodes_.emplace_back(node);

  Eigen::MatrixXd information_matrix = _relative_covariance.inverse();
  addSE3Edge(last_node_, node, _relative_pose, information_matrix);
  last_node_      = node;
  last_node_pose_ = _absolute_pose;
}

std::vector<std::array<double, 7>> GraphG2O::getGraph() {
  std::vector<std::array<double, 7>> pose_graph;
  // Get number of nodes of the graph
  pose_graph.reserve(n_vertices_);

  for (int i = 0; i < n_vertices_; i++) {
    graph_->vertex(i);
    // for (auto p : graph_ptr_->vertices()) {
    for (std::pair<const int, g2o::HyperGraph::Vertex*> p : graph_->vertices()) {
      int id = p.first;
      if (id != i) continue;

      auto node = dynamic_cast<g2o::VertexSE3*>(p.second);
      if (node) {
        auto T               = node->estimate().translation();
        Eigen::Quaterniond R = Eigen::Quaterniond(node->estimate().rotation());

        pose_graph.emplace_back(
            std::array<double, 7>{T.x(), T.y(), T.z(), R.w(), R.x(), R.y(), R.z()});
      } else {
        WARN("Node is not VertexSE3");
      }
    }
  }
  return pose_graph;
}

void GraphG2O::addNewObjectKeyframe(const std::string _obj_id,
                                    const Eigen::Isometry3d& _obj_absolute_pose,
                                    const Eigen::Isometry3d& _obj_relative_pose,
                                    const Eigen::MatrixXd& _obj_covariance) {
  // Eigen::Isometry3d node_pose = Eigen::Isometry3d::Identity();

  g2o::VertexSE3* object_node;
  int obj_node_id = obj_id2node_[_obj_id];

  // TODO: Check if there is a better way
  if (obj_node_id == 0) {
    auto [t_object_node, id] = addSE3Node(_obj_absolute_pose);
    object_node              = t_object_node;
    obj_nodes_.emplace_back(object_node);
    obj_id2node_[_obj_id] = id;
    FLAG("Added new object ID: " << _obj_id << "::" << id);
    // Other way
    ObjectNodeInfo new_obj_node_info(_obj_id, object_node, _obj_covariance);
    obj_nodes_info_.emplace_back(new_obj_node_info);

  } else {
    // Get object node from list
    INFO("Already detected object ID: " << _obj_id);
    object_node = dynamic_cast<g2o::VertexSE3*>(graph_->vertex(obj_node_id));
  }

  Eigen::MatrixXd information_matrix = _obj_covariance.inverse();
  addSE3Edge(last_node_, object_node, _obj_relative_pose, information_matrix);
  // INFO("Added new edge to object");
}

std::vector<ObjectNodeInfo> GraphG2O::getObjectNodes() { return obj_nodes_info_; }

ObjectNodeInfo::ObjectNodeInfo(const std::string _id,
                               g2o::HyperGraph::Vertex* _node,
                               const Eigen::MatrixXd& _covariance) {
  object_id  = _id;
  node       = _node;
  covariance = _covariance;
}
