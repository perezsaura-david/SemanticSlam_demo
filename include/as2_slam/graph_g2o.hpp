/********************************************************************************************
 *  \file       optimizer_g2o.hpp
 *  \brief      An state estimation server for AeroStack2
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

#ifndef __AS2__GRAPH_G2O_HPP_
#define __AS2__GRAPH_G2O_HPP_

#include "graph_g2o.hpp"
#include "utils/general_utils.hpp"

#include <array>
#include <memory>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>

struct ObjectNodeInfo {
  ObjectNodeInfo(const std::string _id,
                 g2o::HyperGraph::Vertex* _node,
                 const Eigen::MatrixXd& _covariance);
  ~ObjectNodeInfo(){};

  std::string object_id;
  g2o::HyperGraph::Vertex* node;
  Eigen::MatrixXd covariance;
};

class GraphG2O {
public:
  GraphG2O(std::string _name);
  ~GraphG2O(){};
  std::vector<std::array<double, 7>> getGraph();
  std::vector<ObjectNodeInfo> getObjectNodes();
  std::pair<g2o::VertexSE3*, int> addSE3Node(const Eigen::Isometry3d& pose);
  g2o::EdgeSE3* addSE3Edge(g2o::VertexSE3* v1,
                           g2o::VertexSE3* v2,
                           const Eigen::Isometry3d& relative_pose,
                           const Eigen::MatrixXd& information_matrix);
  void optimizeGraph();
  void initGraph(const Eigen::Vector3d& initial_position       = Eigen::Vector3d(0, 0, 0),
                 const Eigen::Quaterniond& initial_orientation = Eigen::Quaterniond(1, 0, 0, 0));
  void addNewKeyframe(const Eigen::Isometry3d& _absolute_pose,
                      const Eigen::Isometry3d& _relative_pose,
                      const Eigen::MatrixXd& _relative_covariance);
  void addNewObjectKeyframe(const std::string _obj_id,
                            const Eigen::Isometry3d& _obj_absolute_pose,
                            const Eigen::Isometry3d& _obj_relative_pose,
                            const Eigen::MatrixXd& _obj_covariance);
  std::shared_ptr<g2o::SparseOptimizer> graph_;  // g2o graph
  g2o::VertexSE3* last_node_;
  Eigen::Isometry3d last_node_pose_;
  std::vector<g2o::HyperGraph::Vertex*> odom_nodes_;
  std::vector<g2o::HyperGraph::Vertex*> obj_nodes_;
  std::unordered_map<std::string, int> obj_id2node_;
  std::vector<ObjectNodeInfo> obj_nodes_info_;

private:
  int n_vertices_ = 0;
  int n_edges_    = 0;
  std::string name_;
};

#endif  // __AS2__OPTIMIZER_G2O_HPP_
