// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file graph_g2o.cpp
 *
 * OptimizerG2O class implementation for AeroStack2
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "as2_slam/graph_g2o.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <Eigen/src/Geometry/Transform.h>
#include <g2o/core/optimization_algorithm_factory.h>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>

#include "as2_slam/graph_edge_types.hpp"
#include "as2_slam/graph_node_types.hpp"
// #include "as2_slam/object_builder.hpp"
#include "as2_slam/object_detection_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

GraphG2O::GraphG2O(std::string _name)
{
  name_ = _name;
  FLAG_GRAPH("Create " << name_);

  graph_ = std::make_shared<g2o::SparseOptimizer>();                   // g2o graph
  std::string solver_type = "lm_var_cholmod";                          // Check list of solver types
  // INFO("construct solver: " << solver_type);
  g2o::OptimizationAlgorithmFactory * solver_factory =
    g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm * solver = solver_factory->construct(solver_type, solver_property);
  graph_->setAlgorithm(solver);

  if (!graph_->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate main solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }

  n_vertices_ = 0;
  n_edges_ = 0;
}

std::string GraphG2O::getName() {return name_;}
std::vector<GraphNode *> GraphG2O::getNodes() {return graph_nodes_;}
std::vector<GraphEdge *> GraphG2O::getEdges() {return graph_edges_;}
std::unordered_map<std::string, GraphNode *> GraphG2O::getObjectNodes() {return obj_id2node_;}
OdomNode * GraphG2O::getLastOdomNode() {return last_odom_node_;}


void GraphG2O::setFixedObjects(const std::vector<IsometryWithID> & _fixed_objects)
{
  // FIXME: testing covariance
  Eigen::Matrix<double, 6, 6> aruco_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;

  for (auto object : _fixed_objects) {
    ArucoNode * fixed_node(new ArucoNode(object.isometry));
    fixed_node->setFixed();
    // fixed_node->setCovariance(aruco_covariance);
    addNode(*fixed_node);
    obj_id2node_[object.id] = fixed_node;
    FLAG_GRAPH("Added fixed object ID: " << object.id);
  }
}

void GraphG2O::initGraph(const Eigen::Isometry3d & _initial_pose)
{
  // this position will help to find the correct solution (like the prior)
  // Initial_pose is set to (0,0,0) by default
  Eigen::Isometry3d node_pose = _initial_pose;
  OdomNode * fixed_node(new OdomNode(node_pose));
  fixed_node->setFixed();
  addNode(*fixed_node);
  last_odom_node_ = fixed_node;
}

void GraphG2O::optimizeGraph()
{
  const int num_iterations = 100;
  INFO_GRAPH("--- optimizing graph ---");
  INFO_GRAPH("nodes: " << graph_->vertices().size() << "   edges: " << graph_->edges().size());
  // std::cout << "optimizing... " << std::flush;
  // std::cout << "init" << std::endl;
  graph_->initializeOptimization();
  graph_->setVerbose(false);

  double chi2 = graph_->chi2();
  if (std::isnan(chi2)) {
    ERROR_GRAPH("GRAPH RETURNED A NAN WAITING AFTER OPTIMIZATION");
  }
  // std::cout << "Start optimization" << std::endl;
  graph_->optimize(num_iterations);
  // int iterations = graph_->optimize(num_iterations);
  FLAG_GRAPH("Optimization done");
  // std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  // std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  if (std::isnan(graph_->chi2())) {
    throw std::invalid_argument("GRAPH RETURNED A NAN...STOPPING THE EXPERIMENT");
  }
}

void GraphG2O::addNode(GraphNode & _node)
{
  // INFO("Add Node to Graph: " << name_);
  int id = n_vertices_++;
  _node.getVertex()->setId(id);
  if (!graph_->addVertex(_node.getVertex())) {
    WARN_GRAPH("Vertex not added");
    return;
  }
  graph_nodes_.emplace_back(&_node);
}

void GraphG2O::addEdge(GraphEdge & _edge)
{
  int id = n_edges_++;
  _edge.getEdge()->setId(id);
  if (!graph_->addEdge(_edge.getEdge())) {
    WARN_GRAPH("Edge not added");
    return;
  }
  graph_edges_.emplace_back(&_edge);
}

void GraphG2O::addNewKeyframe(
  const Eigen::Isometry3d & _absolute_pose,
  const Eigen::Isometry3d & _relative_pose,
  const Eigen::MatrixXd & _relative_covariance)
{
  // DEBUG("LOP: " << last_odom_node_->getPose().translation().transpose());
  OdomNode * odom_node(new OdomNode(_absolute_pose));
  addNode(*odom_node);

  Eigen::MatrixXd information_matrix = _relative_covariance.inverse();
  OdomEdge * odom_edge(new OdomEdge(
      last_odom_node_, odom_node, _relative_pose,
      information_matrix));
  addEdge(*odom_edge);
  last_odom_node_ = odom_node;
}

void GraphG2O::addNewObjectDetection(
  ObjectDetection * _object_detection)
{
  GraphNode * object_node;
  // Get object node from list
  object_node = obj_id2node_[_object_detection->getId()];
  if (!object_node) {
    object_node = _object_detection->createNode();
    addNode(*object_node);

    // ObjectDetection * empty_object_detection = _object_detection->createEmptyDetection();
    // graph_object = GraphObject();
    obj_id2node_[_object_detection->getId()] = object_node;
    FLAG_GRAPH("Added new object ID: " << _object_detection->getId());
  } else {
    INFO_GRAPH("Already detected object ID: " << _object_detection->getId());
  }

  // DEBUG_GRAPH("ADD EDGE TO: " << object_node->getNodeName());
  GraphEdge * object_edge = _object_detection->createEdge(last_odom_node_, object_node);
  addEdge(*object_edge);
  // DEBUG("Added new edge to object");
}

// void GraphG2O::addNewObjectKeyframe(
//   const std::string & _obj_id,
//   const Eigen::Isometry3d & _obj_absolute_pose,
//   const Eigen::Isometry3d & _obj_relative_pose,
//   const Eigen::MatrixXd & _obj_covariance)
// {
//   ArucoNode * object_node;
//   // Get object node from list
//   object_node = obj_id2node_[_obj_id];
//   if (!object_node) {
//     // TODO(dps): Check if there is a better way
//     ArucoNode * aruco_node(new ArucoNode(_obj_absolute_pose));
//     addNode(*aruco_node);
//     aruco_node->setCovariance(_obj_covariance);
//     object_node = aruco_node;

//     obj_id2node_[_obj_id] = object_node;
//     FLAG("Added new object ID: " << _obj_id);
//   } else {
//     INFO("Already detected object ID: " << _obj_id);
//   }

//   Eigen::MatrixXd information_matrix = _obj_covariance.inverse();
//   // DEBUG("ADD EDGE TO: " << name_);
//   ArucoEdge * aruco_edge(
//     new ArucoEdge(last_odom_node_, object_node, _obj_relative_pose, information_matrix));
//   addEdge(*aruco_edge);
//   // DEBUG("Added new edge to object");
// }
