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

#include "as2_slam/optimizer_g2o.hpp"
#include <memory>
#include <utility>
#include "graph_g2o.hpp"
#include "graph_node_types.hpp"
#include "utils/conversions.hpp"
#include "utils/debug_utils.hpp"
#include "utils/general_utils.hpp"

OptimizerG2O::OptimizerG2O() {
  FLAG("STARTING SEMANTIC SLAM");

  main_graph = std::make_shared<GraphG2O>("Main Graph");
  temp_graph = std::make_shared<GraphG2O>("Temp Graph");

  if (odometry_is_relative_) {
    WARN("Relative odometry");
  } else {
    WARN("Absolute odometry");
  }

  main_graph->initGraph();
}

bool OptimizerG2O::handleNewOdom(const PoseSE3& _odom_pose,
                                 const Eigen::MatrixXd& _odom_covariance) {
  Eigen::Isometry3d absolute_odom_pose;
  Eigen::Isometry3d relative_odom_pose;
  if (odometry_is_relative_) {
    // TODO RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
  } else {
    // ABSOLUTE ODOMETRY
    absolute_odom_pose = Eigen::Translation3d(_odom_pose.position) * _odom_pose.orientation;
    relative_odom_pose = main_graph->last_odom_node_->getPose().inverse() * absolute_odom_pose;
  }
  // TODO: check time from the last odometry received

  // Check distance from the last odometry received
  double translation_distance_from_last_node = relative_odom_pose.translation().norm();
  double rotation_distance_from_last_node =
      relative_odom_pose.rotation().norm();  // FIXME: get rotation distance

  if (translation_distance_from_last_node < odometry_distance_threshold_) {
    if (rotation_distance_from_last_node < odometry_orientation_threshold_) {
      // INFO("New odometry distance is not enough: " << translation_distance_from_last_node);
      return false;
    }
  }
  // INFO("New odometry distance is enough: " << translation_distance_from_last_node);

  main_graph->addNewKeyframe(absolute_odom_pose, relative_odom_pose, _odom_covariance);

  if (temp_graph_generated) {
    for (auto object : temp_graph->getObjectNodes()) {
      Eigen::Isometry3d absolute_obj_pose = object.second->getPose();
      Eigen::Isometry3d relative_obj_pose = absolute_odom_pose.inverse() * absolute_obj_pose;
      main_graph->addNewObjectKeyframe(object.first, absolute_obj_pose, relative_obj_pose,
                                       object.second->getCovariance());
    }

    FLAG("RESET TEMP GRAPH");
    auto sharing = temp_graph.use_count();
    if (sharing > 1) DEBUG("Temp graph Shared: " << sharing);
    temp_graph.reset();
    // TODO: Check this
    temp_graph           = std::make_shared<GraphG2O>("Temp Graph");
    temp_graph_generated = false;
  }

  // TODO: Choose when to optimize: either every time a new keyframe is added, or every certain
  // period of time
  main_graph->optimizeGraph();
  // debugGraphVertices(main_graph);

  return true;
}

void OptimizerG2O::handleNewObject(const std::string _obj_id,
                                   const PoseSE3& _obj_pose,
                                   const Eigen::MatrixXd& _obj_covariance,
                                   const PoseSE3& _odom_pose,
                                   const Eigen::MatrixXd& _odom_covariance) {
  Eigen::Isometry3d absolute_odom_pose;
  if (odometry_is_relative_) {
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return;
  } else {
    absolute_odom_pose = Eigen::Translation3d(_odom_pose.position) * _odom_pose.orientation;
  }

  if (!temp_graph_generated) {
    temp_graph->initGraph(absolute_odom_pose);
    temp_graph_generated = true;
    return;
  }

  Eigen::Isometry3d relative_odom_pose;
  if (odometry_is_relative_) {
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return;
  } else {
    relative_odom_pose = temp_graph->last_odom_node_->getPose().inverse() * absolute_odom_pose;
  }

  // check distance from the last odometry received
  double translation_distance_from_last_node = relative_odom_pose.translation().norm();
  if (translation_distance_from_last_node < obj_odometry_distance_threshold_) {
    // std::cout << "New odometry distance is not enough" << std::endl;
    return;
  }
  // DEBUG("**** Adding new ODOM keyframe from Object detection ****");
  temp_graph->addNewKeyframe(absolute_odom_pose, relative_odom_pose, _odom_covariance);

  // DEBUG("**** Added new OBJECT keyframe ****");
  // TODO: CURRENT OBJECT POSITION IS ABSOLUTE. SHOULD OBJECT POSITION BE RELATIVE?
  Eigen::Isometry3d absolute_obj_pose =
      Eigen::Translation3d(_obj_pose.position) * _obj_pose.orientation;
  Eigen::Isometry3d relative_obj_pose = absolute_odom_pose.inverse() * absolute_obj_pose;
  temp_graph->addNewObjectKeyframe(_obj_id, absolute_obj_pose, relative_obj_pose, _obj_covariance);
  temp_graph->optimizeGraph();

  // debugGraphVertices(temp_graph);
}

bool OptimizerG2O::getNodePose(g2o::HyperGraph::Vertex* _node,
                               std::pair<Eigen::Vector3d, Eigen::Quaterniond>& _node_pose) {
  g2o::VertexSE3* node_se3 = dynamic_cast<g2o::VertexSE3*>(_node);
  if (!node_se3) {
    return false;
  }

  _node_pose.first  = node_se3->estimate().translation();
  _node_pose.second = Eigen::Quaterniond(node_se3->estimate().rotation());

  return true;
}
