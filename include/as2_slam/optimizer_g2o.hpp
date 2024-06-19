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

#ifndef __AS2__OPTIMIZER_G2O_HPP_
#define __AS2__OPTIMIZER_G2O_HPP_

#include "graph_g2o.hpp"
#include "utils/general_utils.hpp"

#include <array>
#include <memory>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>

class OptimizerG2O {
public:
  OptimizerG2O();
  ~OptimizerG2O(){};
  std::shared_ptr<GraphG2O> main_graph;
  std::shared_ptr<GraphG2O> temp_graph;

  bool handleNewOdom(const Eigen::Vector3d& _odom_position,
                     const Eigen::Quaterniond& _odom_orientation,
                     const Eigen::MatrixXd& _odom_covariance);
  void handleNewObject(const std::string _obj_id,
                       const Eigen::Vector3d& _obj_position,
                       const Eigen::Quaterniond& _obj_orientation,
                       const Eigen::MatrixXd& _obj_covariance,
                       const Eigen::Vector3d& _odom_position,
                       const Eigen::Quaterniond& _odom_orientation,
                       const Eigen::MatrixXd& _odom_covariance);

  bool getNodePose(g2o::HyperGraph::Vertex* _node,
                   std::pair<Eigen::Vector3d, Eigen::Quaterniond>& _node_pose);
  std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>>> getEdgesLines(
      std::shared_ptr<GraphG2O>& _graph);
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> getNodePoses(
      std::shared_ptr<GraphG2O>& _graph,
      const std::string _mode);

private:
  bool first_odom_          = true;
  bool temp_graph_generated = false;
  // TODO: add time_threshold_
  double translation_distance_from_last_node_ = 0.0;
  double rotation_distance_from_last_node_    = 0.0;

  Eigen::Vector3d absolute_odom_position;
  Eigen::Quaterniond absolute_odom_orientation;

  // PARAMETERS
  double odometry_distance_threshold_        = 1.0;  // meters
  double odometry_orientation_threshold_     = 2;    // radians
  double obj_odometry_distance_threshold_    = 0.5;  // meters
  double obj_odometry_orientation_threshold_ = 0.5;  // radians
  bool odometry_is_relative_                 = false;
};

#endif  // __AS2__OPTIMIZER_G2O_HPP_
