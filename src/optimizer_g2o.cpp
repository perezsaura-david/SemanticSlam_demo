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

OptimizerG2O::OptimizerG2O() {
  FLAG("STARTING SEMANTIC SLAM");

  main_graph = std::make_shared<GraphG2O>("Main graph");
  temp_graph = std::make_shared<GraphG2O>("Temp graph");

  if (odometry_is_relative_) {
    WARN("Relative odometry");
  } else {
    WARN("Absolute odometry");
  }

  main_graph->initGraph();
}

std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>>>
OptimizerG2O::getEdgesLines(std::shared_ptr<GraphG2O>& _graph) {
  std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>>> edge_poses;

  auto edges = _graph->graph_->edges();

  for (auto edge : edges) {
    auto vertices = edge->vertices();
    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> node_poses;
    for (auto vertex : vertices) {
      // std::cout << "ID: " << vertex->id() << std::endl;
      std::pair<Eigen::Vector3d, Eigen::Quaterniond> node_pose;
      if (getNodePose(vertex, node_pose)) {
        node_poses.emplace_back(node_pose);
      } else
        ERROR("CAN'T CONVERT VERTEX TO SE3");
    }
    edge_poses.emplace_back(node_poses);
  }

  return edge_poses;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> OptimizerG2O::getNodePoses(
    std::shared_ptr<GraphG2O>& _graph,
    const std::string _mode) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> node_poses;
  std::vector<g2o::HyperGraph::Vertex*>* nodes;
  if (_mode == "odom")
    nodes = &_graph->odom_nodes_;
  else if (_mode == "obj")
    nodes = &_graph->obj_nodes_;
  else {
    std::cerr << "GetObj:Mode not recognized" << std::endl;
    return node_poses;
  }
  // std::cout << "OBJ_NODES_SIZE: " << nodes->size() << std::endl;
  node_poses.reserve(nodes->size());

  for (auto node : *nodes) {
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> node_pose;
    if (getNodePose(node, node_pose)) {
      node_poses.emplace_back(node_pose);
    } else
      ERROR("CAN'T CONVERT VERTEX TO SE3");
  }
  return node_poses;
}

bool OptimizerG2O::handleNewOdom(const Eigen::Vector3d& _odom_position,
                                 const Eigen::Quaterniond& _odom_orientation,
                                 const Eigen::MatrixXd& _odom_covariance) {
  Eigen::Isometry3d absolute_odom_pose;
  Eigen::Isometry3d relative_odom_pose;
  // Eigen::Isometry3d odom_pose = Eigen::Translation3d(_odom_position) * _odom_orientation;
  if (odometry_is_relative_) {
    // TODO RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
  } else {
    // ABSOLUTE ODOMETRY
    absolute_odom_pose = Eigen::Translation3d(_odom_position) * _odom_orientation;
    relative_odom_pose = main_graph->last_node_pose_.inverse() * absolute_odom_pose;
  }
  // TODO: check time from the last odometry received

  // check distance from the last odometry received
  double translation_distance_from_last_node = relative_odom_pose.translation().norm();
  double rotation_distance_from_last_node =
      relative_odom_pose.rotation().norm();  // FIXME: get rotation distance

  if (translation_distance_from_last_node < odometry_distance_threshold_) {
    if (rotation_distance_from_last_node < odometry_orientation_threshold_) {
      // Pose distance is not enough to create a new node
      // std::cout << "New odometry distance is not enough" << std::endl;
      return false;
    }
  }

  main_graph->addNewKeyframe(absolute_odom_pose, relative_odom_pose, _odom_covariance);

  if (temp_graph_generated) {
    for (auto obj_node_info : temp_graph->getObjectNodes()) {
      std::pair<Eigen::Vector3d, Eigen::Quaterniond> obj_node_pose;
      if (getNodePose(obj_node_info.node, obj_node_pose)) {
        // TODO: IS THIS THE RIGHT WAY?
        Eigen::Isometry3d absolute_obj_pose =
            Eigen::Translation3d(obj_node_pose.first) * obj_node_pose.second;
        Eigen::Isometry3d relative_obj_pose = absolute_odom_pose.inverse() * absolute_obj_pose;
        // Eigen::Matrix<double, 6, 6> obj_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;
        main_graph->addNewObjectKeyframe(obj_node_info.object_id, absolute_obj_pose,
                                         relative_obj_pose, obj_node_info.covariance);
      } else
        ERROR("CAN'T CONVERT OBJECT VERTEX TO SE3");
    }
    // TODO: RESET TEMP_GRAPH
    FLAG("RESET TEMP GRAPH");
    auto sharing = temp_graph.use_count();
    if (sharing > 1) DEBUG("Temp graph Shared: " << sharing);
    temp_graph.reset();
    temp_graph           = std::make_shared<GraphG2O>("Temp Graph");
    temp_graph_generated = false;
  }

  // TODO: Choose when to optimize: either every time a new keyframe is added, or every certain
  // period of time
  main_graph->optimizeGraph();

  // DEBUG
  // for (auto p : main_graph->graph_->vertices()) {
  //   // for (pair[ int id, VertexSE3 node ] : graph->vertices()) {
  //   int id    = p.first;
  //   auto node = dynamic_cast<g2o::VertexSE3*>(p.second);
  //   if (node) {
  //     auto T = node->estimate().translation().transpose();
  //     std::cout << "NODE " << id << " : " << T << std::endl;
  //   }
  //   // else { Node is not VertexSE3 }
  // }
  //
  return true;
}

void OptimizerG2O::handleNewObject(const std::string _obj_id,
                                   const Eigen::Vector3d& _obj_position,
                                   const Eigen::Quaterniond& _obj_orientation,
                                   const Eigen::MatrixXd& _obj_covariance,
                                   const Eigen::Vector3d& _odom_position,
                                   const Eigen::Quaterniond& _odom_orientation,
                                   const Eigen::MatrixXd& _odom_covariance) {
  Eigen::Isometry3d absolute_odom_pose;
  Eigen::Isometry3d relative_odom_pose;
  if (odometry_is_relative_) {
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return;
  } else {
    // ABSOLUTE ODOMETRY
    absolute_odom_pose = Eigen::Translation3d(_odom_position) * _odom_orientation;
    relative_odom_pose = temp_graph->last_node_pose_.inverse() * absolute_odom_pose;
  }

  if (!temp_graph_generated) {
    // DEBUG("**** Adding initial ODOM keyframe from Object detection ****");
    temp_graph->initGraph(_odom_position, _odom_orientation);
    temp_graph_generated = true;
    // return;
  } else {
    // check distance from the last odometry received
    double translation_distance_from_last_node = relative_odom_pose.translation().norm();

    if (translation_distance_from_last_node < obj_odometry_distance_threshold_) {
      // std::cout << "New odometry distance is not enough" << std::endl;
      return;
    }

    // DEBUG("**** Adding new ODOM keyframe from Object detection ****");
    temp_graph->addNewKeyframe(absolute_odom_pose, relative_odom_pose, _odom_covariance);
  }

  // DEBUG
  // for (auto p : temp_graph->graph_->vertices()) {
  //   // for (pair[ int id, VertexSE3 node ] : graph->vertices()) {
  //   int id    = p.first;
  //   auto node = dynamic_cast<g2o::VertexSE3*>(p.second);
  //   if (node) {
  //     auto T = node->estimate().translation().transpose();
  //     INFO("NODE " << id << " : " << T);
  //   }
  //   // else { Node is not VertexSE3 }
  // }

  // DEBUG("**** Added new OBJECT keyframe ****");
  // TODO: CURRENT OBJECT POSITION IS ABSOLUTE. SHOULD OBJECT POSITION BE RELATIVE?
  Eigen::Isometry3d absolute_obj_pose = Eigen::Translation3d(_obj_position) * _obj_orientation;
  Eigen::Isometry3d relative_obj_pose = absolute_odom_pose.inverse() * absolute_obj_pose;
  temp_graph->addNewObjectKeyframe(_obj_id, absolute_obj_pose, relative_obj_pose, _obj_covariance);

  temp_graph->optimizeGraph();
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
