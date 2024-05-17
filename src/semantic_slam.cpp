/********************************************************************************************
 *  \file       as2_slam.cpp
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

#include "as2_slam/semantic_slam.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <Eigen/src/Geometry/Translation.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <array>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/duration.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

SemanticSlam::SemanticSlam() : as2::Node("semantic_slam") {
  std::string default_odom_topic       = "cf0/sensor_measurements/noisy_odom";
  std::string default_aruco_pose_topic = "cf0/detect_aruco_markers_behavior/aruco_pose";
  std::string default_reference_frame  = "cf0/map";
  std::string default_robot_frame      = "cf0/base_link";

  std::string default_graph_topic          = "cf0/semantic_slam/graph";
  std::string default_odom_nodes_topic     = "slam/odom_nodes";
  std::string default_obj_odom_nodes_topic = "slam/obj_odom_nodes";
  std::string default_obj_nodes_topic      = "slam/obj_nodes";

  std::shared_ptr<const rclcpp::QoS> reliable_qos =
      std::make_shared<const rclcpp::QoS>(rclcpp::QoS(2));
  std::shared_ptr<const rclcpp::QoS> sensor_qos =
      std::make_shared<const rclcpp::QoS>(rclcpp::SensorDataQoS());

  std::string odom_topic = this->declare_parameter("odom_topic", default_odom_topic);
  std::string aruco_pose_topic =
      this->declare_parameter("aruco_pose_topic", default_aruco_pose_topic);

  std::string graph_topic = this->declare_parameter("graph_topic", default_graph_topic);
  std::string odom_nodes_topic =
      this->declare_parameter("odom_nodes_topic", default_odom_nodes_topic);
  std::string obj_odom_nodes_topic =
      this->declare_parameter("obj_odom_nodes_topic", default_obj_odom_nodes_topic);
  std::string obj_nodes_topic = this->declare_parameter("obj_nodes_topic", default_obj_nodes_topic);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, *sensor_qos, std::bind(&SemanticSlam::odomCallback, this, std::placeholders::_1));

  aruco_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
      aruco_pose_topic, *sensor_qos,
      std::bind(&SemanticSlam::arucoPoseCallback, this, std::placeholders::_1));

  reference_frame_ =
      this->declare_parameter<std::string>("reference_frame", default_reference_frame);
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", default_robot_frame);

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  graph_pub_ = this->create_publisher<nav_msgs::msg::Path>(graph_topic, *reliable_qos);
  odom_nodes_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(odom_nodes_topic, *reliable_qos);
  obj_odom_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      obj_odom_nodes_topic, *reliable_qos);
  obj_nodes_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(obj_nodes_topic, *reliable_qos);
  optimizer_ptr_ = std::make_unique<OptimizerG2O>();
  // = std::make_unique<g2o::SparseOptimizer>();
}

void SemanticSlam::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Odom received: '%s'", msg->header.frame_id.c_str());
  Eigen::Vector3d odom_position;
  Eigen::Quaterniond odom_orientation;

  odom_position.x()    = msg->pose.pose.position.x;
  odom_position.y()    = msg->pose.pose.position.y;
  odom_position.z()    = msg->pose.pose.position.z;
  odom_orientation.w() = msg->pose.pose.orientation.w;
  odom_orientation.x() = msg->pose.pose.orientation.x;
  odom_orientation.y() = msg->pose.pose.orientation.y;
  odom_orientation.z() = msg->pose.pose.orientation.z;
  // Assuming that the covariance is stored in row-major order
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> odom_covariance(
      msg->pose.covariance.data());

  last_odom_abs_position_received_    = odom_position;
  last_odom_abs_orientation_received_ = odom_orientation;
  last_odom_abs_covariance_received_  = odom_covariance;

  // return;

  // TODO: Define how to use this
  // msg->header.frame_id;
  // msg->header.stamp;

  // TODO: handle covariance
  optimizer_ptr_->handleNewOdom(odom_position, odom_orientation, odom_covariance);

  // VISUALIZATION MSGS
  std::vector<std::array<double, 7>> graph = optimizer_ptr_->getGraph();
  nav_msgs::msg::Path graph_msg            = generateGraphMsg(graph);
  graph_pub_->publish(graph_msg);
  visualization_msgs::msg::MarkerArray odom_nodes_msg = generateOdomNodesMsg();
  odom_nodes_pub_->publish(odom_nodes_msg);
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateOdomNodesMsg() {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes_poses =
      optimizer_ptr_->getOdomNodePoses();
  visualization_msgs::msg::MarkerArray nodes_markers_msg;
  // std::cout << "REFERENCE FRAME :" << reference_frame_ << std::endl;
  // nodes_markers_msg.header.frame_id = reference_frame_;
  int counter = 0;
  for (auto node_pose : odom_nodes_poses) {
    visualization_msgs::msg::Marker node_marker_msg;
    node_marker_msg.type = node_marker_msg.ARROW;
    node_marker_msg.id   = counter;
    counter++;
    node_marker_msg.header.frame_id    = reference_frame_;
    node_marker_msg.pose.position.x    = node_pose.first.x();
    node_marker_msg.pose.position.y    = node_pose.first.y();
    node_marker_msg.pose.position.z    = node_pose.first.z();
    node_marker_msg.pose.orientation.w = node_pose.second.w();
    node_marker_msg.pose.orientation.x = node_pose.second.x();
    node_marker_msg.pose.orientation.y = node_pose.second.y();
    node_marker_msg.pose.orientation.z = node_pose.second.z();
    node_marker_msg.scale.x            = 0.5;
    node_marker_msg.scale.y            = 0.05;
    node_marker_msg.scale.z            = 0.05;
    node_marker_msg.color.b            = 1.0;
    node_marker_msg.color.a            = 1.0;
    nodes_markers_msg.markers.emplace_back(node_marker_msg);
  }
  return nodes_markers_msg;
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateObjOdomNodesMsg() {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes_poses =
      optimizer_ptr_->getObjNodePoses("odom");
  visualization_msgs::msg::MarkerArray nodes_markers_msg;
  // std::cout << "REFERENCE FRAME :" << reference_frame_ << std::endl;
  // nodes_markers_msg.header.frame_id = reference_frame_;
  int counter = 0;
  for (auto node_pose : odom_nodes_poses) {
    visualization_msgs::msg::Marker node_marker_msg;
    node_marker_msg.type = node_marker_msg.ARROW;
    node_marker_msg.id   = counter;
    counter++;
    node_marker_msg.header.frame_id    = reference_frame_;
    node_marker_msg.pose.position.x    = node_pose.first.x();
    node_marker_msg.pose.position.y    = node_pose.first.y();
    node_marker_msg.pose.position.z    = node_pose.first.z();
    node_marker_msg.pose.orientation.w = node_pose.second.w();
    node_marker_msg.pose.orientation.x = node_pose.second.x();
    node_marker_msg.pose.orientation.y = node_pose.second.y();
    node_marker_msg.pose.orientation.z = node_pose.second.z();
    node_marker_msg.scale.x            = 0.5;
    node_marker_msg.scale.y            = 0.05;
    node_marker_msg.scale.z            = 0.05;
    node_marker_msg.color.g            = 1.0;
    node_marker_msg.color.a            = 1.0;
    nodes_markers_msg.markers.emplace_back(node_marker_msg);
  }
  return nodes_markers_msg;
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateObjNodesMsg() {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes_poses =
      optimizer_ptr_->getObjNodePoses("obj");
  visualization_msgs::msg::MarkerArray nodes_markers_msg;
  // std::cout << "REFERENCE FRAME :" << reference_frame_ << std::endl;
  // nodes_markers_msg.header.frame_id = reference_frame_;
  int counter = 0;
  for (auto node_pose : odom_nodes_poses) {
    visualization_msgs::msg::Marker node_marker_msg;
    node_marker_msg.type = node_marker_msg.ARROW;
    node_marker_msg.id   = counter;
    counter++;
    node_marker_msg.header.frame_id    = reference_frame_;
    node_marker_msg.pose.position.x    = node_pose.first.x();
    node_marker_msg.pose.position.y    = node_pose.first.y();
    node_marker_msg.pose.position.z    = node_pose.first.z();
    node_marker_msg.pose.orientation.w = node_pose.second.w();
    node_marker_msg.pose.orientation.x = node_pose.second.x();
    node_marker_msg.pose.orientation.y = node_pose.second.y();
    node_marker_msg.pose.orientation.z = node_pose.second.z();
    node_marker_msg.scale.x            = 0.5;
    node_marker_msg.scale.y            = 0.05;
    node_marker_msg.scale.z            = 0.05;
    node_marker_msg.color.r            = 1.0;
    node_marker_msg.color.a            = 1.0;
    nodes_markers_msg.markers.emplace_back(node_marker_msg);
  }
  return nodes_markers_msg;
}

nav_msgs::msg::Path SemanticSlam::generateGraphMsg(
    const std::vector<std::array<double, 7>>& _graph) {
  nav_msgs::msg::Path graph_msg;
  // FIXME: Get the frame id
  graph_msg.header.frame_id = reference_frame_;

  for (auto pose : _graph) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x    = pose[0];
    pose_msg.pose.position.y    = pose[1];
    pose_msg.pose.position.z    = pose[2];
    pose_msg.pose.orientation.w = pose[3];
    pose_msg.pose.orientation.x = pose[4];
    pose_msg.pose.orientation.y = pose[5];
    pose_msg.pose.orientation.z = pose[6];

    graph_msg.poses.emplace_back(pose_msg);
  }

  return graph_msg;
}

// std::pair<Eigen::Vector3d, Eigen::Quaterniond> SemanticSlam::transformPose(
//     const std::pair<Eigen::Vector3d, Eigen::Quaterniond>& pose,
//     const std::string _reference_frame) {
//   geometry_msgs::msg::TransformStamped ref_frame_transform;
//   try {
//     std::cout << "Transformation between " << reference_frame_ << " and " << ref_frame <<
//     std::endl; ref_frame_transform =
//         tf_buffer_->lookupTransform(reference_frame_, ref_frame, target_ts, tf_timeout);
//     geometry_msgs::msg::PoseStamped transformed_pose;
//     tf2::doTransform(msg->pose, transformed_pose, ref_frame_transform);
//     aruco_position.x()    = transformed_pose.pose.position.x;
//     aruco_position.y()    = transformed_pose.pose.position.y;
//     aruco_position.z()    = transformed_pose.pose.position.z;
//     aruco_orientation.w() = transformed_pose.pose.orientation.w;
//     aruco_orientation.x() = transformed_pose.pose.orientation.x;
//     aruco_orientation.y() = transformed_pose.pose.orientation.y;
//     aruco_orientation.z() = transformed_pose.pose.orientation.z;

//   } catch (const tf2::TransformException& ex) {
//     RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", ref_frame.c_str(),
//                 reference_frame_.c_str(), ex.what());
//     return;
//   }
// }

void SemanticSlam::arucoPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Aruco received: '%s'", msg->id.c_str());
  Eigen::Vector3d aruco_position;
  Eigen::Quaterniond aruco_orientation;
  std::string aruco_id;

  return;

  static int counter = 0;

  counter += 1;
  if (counter < 10) {
    return;  // NO ARUCO DETECTION
  }
  counter = 0;

  aruco_id = msg->id;

  // std::cout << "P__: " << aruco_position << std::endl;
  // std::cout << "O__: " << aruco_orientation << std::endl;

  std::string ref_frame = msg->pose.header.frame_id;

  auto target_ts = tf2::TimePointZero;
  // auto target_ts = msg->pose.header.stamp;
  std::chrono::nanoseconds tf_timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0));

  auto tf_names = tf_buffer_->getAllFrameNames();

  // FIXME: We need this in simulation because of the current optical link implementation in
  // gazebo
  for (auto tf_name : tf_names) {
    if (tf_name.find(ref_frame) < tf_name.length()) {
      ref_frame = tf_name;
      std::cout << "Ref frame changed to: " << ref_frame << std::endl;
    }
  }

  if (ref_frame != reference_frame_) {
    geometry_msgs::msg::TransformStamped ref_frame_transform;
    try {
      // TODO: Make a function for this
      std::cout << "Transformation between " << reference_frame_ << " and " << ref_frame
                << std::endl;
      ref_frame_transform =
          tf_buffer_->lookupTransform(reference_frame_, ref_frame, target_ts, tf_timeout);
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(msg->pose, transformed_pose, ref_frame_transform);
      aruco_position.x()    = transformed_pose.pose.position.x;
      aruco_position.y()    = transformed_pose.pose.position.y;
      aruco_position.z()    = transformed_pose.pose.position.z;
      aruco_orientation.w() = transformed_pose.pose.orientation.w;
      aruco_orientation.x() = transformed_pose.pose.orientation.x;
      aruco_orientation.y() = transformed_pose.pose.orientation.y;
      aruco_orientation.z() = transformed_pose.pose.orientation.z;

    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", ref_frame.c_str(),
                  reference_frame_.c_str(), ex.what());
      return;
    }
  } else {
    aruco_position.x()    = msg->pose.pose.position.x;
    aruco_position.y()    = msg->pose.pose.position.y;
    aruco_position.z()    = msg->pose.pose.position.z;
    aruco_orientation.w() = msg->pose.pose.orientation.w;
    aruco_orientation.x() = msg->pose.pose.orientation.x;
    aruco_orientation.y() = msg->pose.pose.orientation.y;
    aruco_orientation.z() = msg->pose.pose.orientation.z;
  }

  std::cout << "PPP: " << aruco_position << std::endl;
  std::cout << "OOO: " << aruco_orientation << std::endl;

  // Covariance
  // Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> odom_covariance(
  //     msg->pose.covariance.data());

  // TODO: Define how to use this
  // msg->pose.header.frame_id;
  // msg->pose.header.stamp;
  std::cout << "ARUCO: add last odom" << std::endl;
  // Eigen::Matrix<double, 6, 6>(),
  Eigen::Matrix<double, 6, 6> aruco_covariance = Eigen::MatrixXd::Identity(6, 6);

  std::cout << "ARUCO: add object" << std::endl;
  optimizer_ptr_->handleNewObject(
      aruco_position, aruco_orientation, aruco_covariance, last_odom_abs_position_received_,
      last_odom_abs_orientation_received_, last_odom_abs_covariance_received_);

  // VISUALIZATION MSGS
  visualization_msgs::msg::MarkerArray obj_odom_nodes_msg = generateObjOdomNodesMsg();
  obj_odom_nodes_pub_->publish(obj_odom_nodes_msg);
  visualization_msgs::msg::MarkerArray obj_nodes_msg = generateObjOdomNodesMsg();
  obj_nodes_pub_->publish(obj_nodes_msg);
}

//*********************************************************************************************//
//*********************************************************************************************//
//*********************************************************************************************//

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

OptimizerG2O::OptimizerG2O() {
  std::string solver_type = "lm_var_cholmod";  // Check list of solver types
  graph_ptr_              = std::make_shared<g2o::SparseOptimizer>();
  // g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_ptr_.get());
  temp_graph_ptr_ = std::make_unique<g2o::SparseOptimizer>();
  // g2o::SparseOptimizer* temp_graph = dynamic_cast<g2o::SparseOptimizer*>(temp_graph_ptr_.get());

  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph_ptr_->setAlgorithm(solver);
  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory_temp =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property_temp;
  g2o::OptimizationAlgorithm* solver_temp = solver_factory->construct(solver_type, solver_property);
  temp_graph_ptr_->setAlgorithm(solver_temp);

  if (!graph_ptr_->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  if (odometry_is_relative_) {
    std::cout << "Relative odometry" << std::endl;
  } else {
    std::cout << "Absolute odometry" << std::endl;
  }

  initGraph(graph_ptr_);
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> OptimizerG2O::getOdomNodePoses() {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes;
  // std::cout << "ODOM_NODES_SIZE: " << odom_nodes_.size() << std::endl;
  odom_nodes.reserve(odom_nodes_.size());

  for (auto node : odom_nodes_) {
    // for (int i = 0; i < int(odom_nodes_.size()); i++) {
    // auto se3node = dynamic_cast<g2o::VertexSE3*>(odom_nodes_[i]);
    //
    // auto se3node = dynamic_cast<g2o::VertexSE3*>(node);
    // if (se3node) else
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> node_pose;
    if (getNodePose(node, node_pose)) {
      // std::cout << "CONVERTED" << std::endl;
      // std::cout << node_pose.first << std::endl;
      odom_nodes.emplace_back(node_pose);
    } else
      std::cout << "CAN'T CONVERT VERTEX TO SE3" << std::endl;
  }
  return odom_nodes;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> OptimizerG2O::getObjNodePoses(
    const std::string _mode) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> node_poses;
  std::vector<g2o::HyperGraph::Vertex*>* nodes;
  if (_mode == "odom")
    nodes = &obj_odom_nodes_;
  else if (_mode == "obj")
    nodes = &obj_nodes_;
  else {
    std::cerr << "GetObj:Mode not recognized" << std::endl;
    return node_poses;
  }
  std::cout << "OBJ_NODES_SIZE: " << nodes->size() << std::endl;
  node_poses.reserve(nodes->size());

  for (auto node : *nodes) {
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> node_pose;
    if (getNodePose(node, node_pose)) {
      node_poses.emplace_back(node_pose);
    } else
      std::cout << "CAN'T CONVERT VERTEX TO SE3" << std::endl;
  }
  return node_poses;
}

std::pair<g2o::VertexSE3*, int> OptimizerG2O::addSE3Node(
    const Eigen::Isometry3d& _pose,
    std::shared_ptr<g2o::SparseOptimizer> _graph) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  auto id = n_vertices_++;
  vertex->setId(static_cast<int>(id));
  vertex->setEstimate(_pose);
  if (!_graph->addVertex(vertex)) {
    // if (!graph_ptr_->addVertex(vertex)) {
    std::cerr << "Vertex not added" << std::endl;
  }
  return {vertex, id};
}

g2o::EdgeSE3* OptimizerG2O::addSE3Edge(g2o::VertexSE3* _v1,
                                       g2o::VertexSE3* _v2,
                                       const Eigen::Isometry3d& _relative_pose,
                                       const Eigen::MatrixXd& _information_matrix,
                                       std::shared_ptr<g2o::SparseOptimizer> _graph) {
  // Check Information Matrix
  // std::cout << "information_matrix" << std::endl;
  // std::cout << information_matrix << std::endl;

  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setId(static_cast<int>(n_edges_++));
  edge->setMeasurement(_relative_pose);
  edge->setInformation(_information_matrix);
  edge->vertices()[0] = _v1;
  edge->vertices()[1] = _v2;
  if (!_graph->addEdge(edge)) {
    // if (!graph_ptr_->addEdge(edge)) {
    std::cerr << "Edge not added" << std::endl;
  }
  return edge;
}

void OptimizerG2O::optimizeGraph(std::shared_ptr<g2o::SparseOptimizer> _graph) {
  // g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_ptr_.get());
  const int num_iterations = 100;

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << _graph->vertices().size() << "   edges: " << _graph->edges().size()
            << std::endl;
  // std::cout << "optimizing... " << std::flush;

  // std::cout << "init" << std::endl;
  _graph->initializeOptimization();
  _graph->setVerbose(false);

  double chi2 = _graph->chi2();
  if (std::isnan(chi2)) {
    std::cout << "GRAPH RETURNED A NAN WAITING AFTER OPTIMIZATION" << std::endl;
  }
  std::cout << "Start optimization" << std::endl;
  int iterations = _graph->optimize(num_iterations);
  std::cout << "Optimization done" << std::endl;
  // std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  // std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  if (std::isnan(_graph->chi2())) {
    throw std::invalid_argument("GRAPH RETURNED A NAN...STOPPING THE EXPERIMENT");
  }
}

void OptimizerG2O::initGraph(std::shared_ptr<g2o::SparseOptimizer> _graph,
                             const Eigen::Vector3d& initial_position,
                             const Eigen::Quaterniond& initial_orientation) {
  // this position will help the optimizer to find the correct solution ( its like the
  // prior )
  // Initial_pose is set to (0,0,0) by default
  Eigen::Isometry3d node_pose = Eigen::Translation3d(initial_position) * initial_orientation;
  // auto [ground, id]           = addSE3Node(Eigen::Isometry3d::Identity());
  auto [ground, id] = addSE3Node(node_pose, _graph);
  odom_nodes_.emplace_back(ground);
  ground->setFixed(true);
  last_node_      = ground;
  last_node_pose_ = node_pose;
}

void OptimizerG2O::addNewKeyframe(const Eigen::Isometry3d& _absolute_pose,
                                  const Eigen::Isometry3d& _relative_pose,
                                  const Eigen::MatrixXd& _relative_covariance) {
  std::cout << "*** NEW KEY FRAME ***" << std::endl;
  auto [node, id] = addSE3Node(_absolute_pose, graph_ptr_);
  odom_nodes_.emplace_back(node);

  addSE3Edge(last_node_, node, _relative_pose, _relative_covariance, graph_ptr_);
  last_node_ = node;
}

// DEPRECATED
void OptimizerG2O::addNewKeyframe(const Eigen::Vector3d& _relative_translation,
                                  const Eigen::Quaterniond& _relative_rotation,
                                  const Eigen::MatrixXd& _relative_covariance) {
  // Change Identity to Odom pose?
  // Eigen::Isometry3d node_pose = Eigen::Isometry3d::Identity();
  std::cout << "*** NEW KEY FRAME ***" << std::endl;
  Eigen::Isometry3d node_pose =
      Eigen::Translation3d(absolute_odom_position) * absolute_odom_orientation;

  std::cout << "Node: " << absolute_odom_position << std::endl;
  std::cout << "Node: " << absolute_odom_orientation << std::endl;

  auto [node, id] = addSE3Node(node_pose, graph_ptr_);
  odom_nodes_.emplace_back(node);

  // Eigen::Isometry3d relative_pose;
  // relative_pose.setIdentity();
  // relative_pose.translation() = relative_position;
  Eigen::Isometry3d relative_pose =
      Eigen::Translation3d(_relative_translation) * _relative_rotation;
  // auto last_node                  = graph_ptr_->vertices()[1];
  // auto prev_node                  = dynamic_cast<g2o::VertexSE3*>(last_node);
  //
  std::cout << "Edge: " << _relative_translation << std::endl;
  std::cout << "Edge: " << _relative_rotation << std::endl;
  // auto prev_node = last_node_;
  // Eigen::MatrixXd::Identity(6, 6);
  std::cout << "last_node " << last_node_ << std::endl;
  std::cout << "new_node " << node << std::endl;
  addSE3Edge(last_node_, node, relative_pose, _relative_covariance, graph_ptr_);
  // addSE3Edge(last_node_, node, relative_pose, Eigen::MatrixXd::Identity(6, 6));
  last_node_ = node;
}

void OptimizerG2O::handleNewOdom(const Eigen::Vector3d& _odom_position,
                                 const Eigen::Quaterniond& _odom_orientation,
                                 const Eigen::MatrixXd& _odom_covariance) {
  Eigen::Isometry3d absolute_odom_pose;
  Eigen::Isometry3d relative_odom_pose;
  // Eigen::Isometry3d odom_pose = Eigen::Translation3d(_odom_position) * _odom_orientation;
  if (odometry_is_relative_) {
    // TODO RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    std::cerr << "RELATIVE ODOMETRY NOT IMPLEMENTED" << std::endl;
  } else {
    // ABSOLUTE ODOMETRY
    absolute_odom_pose = Eigen::Translation3d(_odom_position) * _odom_orientation;
    relative_odom_pose = last_node_pose_.inverse() * absolute_odom_pose;
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
      return;
    }
  }

  // TODO: Check to do this inside the next function
  last_node_pose_ = absolute_odom_pose;

  addNewKeyframe(absolute_odom_pose, relative_odom_pose, _odom_covariance);

  // TODO: Choose when to optimize: either every time a new keyframe is added, or every certain
  // period of time
  optimizeGraph(graph_ptr_);

  for (auto p : graph_ptr_->vertices()) {
    // for (pair[ int id, VertexSE3 node ] : graph->vertices()) {
    int id    = p.first;
    auto node = dynamic_cast<g2o::VertexSE3*>(p.second);
    if (node) {
      auto T = node->estimate().translation().transpose();
      std::cout << "NODE " << id << " : " << T << std::endl;
    }
    // else { Node is not VertexSE3 }
  }
}

// void OptimizerG2O::handleNewOdom(const Eigen::Vector3d& _odom_position,
//                                  const Eigen::Quaterniond& _odom_orientation,
//                                  const Eigen::MatrixXd& _odom_covariance) {
//   // if (first_odom_) {
//   //   initGraph(_odom_position, _odom_orientation);
//   //   first_odom_ = false;
//   //   return;
//   // }
//   // std::cout << "Handling new odometry" << std::endl;
//   static Eigen::Vector3d translation_from_last_node;
//   static Eigen::Quaterniond rotation_from_last_node;
//   Eigen::Vector3d relative_position;
//   Eigen::Quaterniond relative_orientation;

//   // TODO: Find the best way to get the orientation

//   // TODO: Check the best way to do this
//   if (odometry_is_relative_) {
//     // RELATIVE ODOMETRY
//     relative_position         = _odom_position;
//     relative_orientation      = _odom_orientation;
//     absolute_odom_position    = absolute_odom_position + _odom_position;
//     absolute_odom_orientation = absolute_odom_orientation * _odom_orientation;
//     absolute_odom_orientation.normalize();
//     // Calculate position and orientation from last node
//     // Option 1: Sum up every relative step
//     translation_from_last_node = translation_from_last_node + relative_position;
//     rotation_from_last_node    = rotation_from_last_node * _odom_orientation;
//     rotation_from_last_node.normalize();
//   } else {
//     // ABSOLUTE ODOMETRY
//     //
//     // relative_position    = _odom_position - absolute_odom_position;
//     // relative_orientation = _odom_orientation * absolute_odom_orientation.inverse();
//     // relative_orientation.normalize();

//     absolute_odom_position    = _odom_position;
//     absolute_odom_orientation = _odom_orientation;

//     // Calculate position and orientation from last node
//     // Option 2: Substract last node pose from the current one
//     translation_from_last_node = absolute_odom_position - last_node_position_;
//     rotation_from_last_node    = absolute_odom_orientation * last_node_orientation_.inverse();
//     // rotation_from_last_node = last_node_orientation_.inverse() * absolute_odom_orientation;
//     rotation_from_last_node.normalize();
//   }
//   // Calculate position and orientation from last node
//   // Option 3: Sum up the distance of every step (f abs(distance) you can sum up steps in
//   // different directions)

//   // TODO: check time from the last odometry received

//   // check distance from the last odometry received
//   double translation_distance_from_last_node = translation_from_last_node.norm();
//   double rotation_distance_from_last_node    = rotation_from_last_node.norm();
//   // std::cout << "T distance: " << translation_distance_from_last_node << std::endl;
//   // std::cout << "R distance: " << rotation_distance_from_last_node << std::endl;
//   if (translation_distance_from_last_node < odometry_distance_threshold_) {
//     if (rotation_distance_from_last_node < odometry_orientation_threshold_) {
//       // Pose distance is not enough to create a new node
//       // std::cout << "New odometry distance is not enough" << std::endl;
//       return;
//     }
//   }

//   last_node_position_    = absolute_odom_position;
//   last_node_orientation_ = absolute_odom_orientation;

//   addNewKeyframe(translation_from_last_node, rotation_from_last_node, _odom_covariance);

//   // TODO: Choose when to optimize: either every time a new keyframe is added, or every certain
//   // period of time
//   optimizeGraph();

//   for (auto p : graph_ptr_->vertices()) {
//     // for (pair[ int id, VertexSE3 node ] : graph->vertices()) {
//     int id    = p.first;
//     auto node = dynamic_cast<g2o::VertexSE3*>(p.second);
//     if (node) {
//       auto T = node->estimate().translation().transpose();
//       std::cout << "NODE " << id << " : " << T << std::endl;
//     }
//     // else { Node is not VertexSE3 }
//   }
// }

void OptimizerG2O::handleNewObject(const Eigen::Vector3d& _obj_position,
                                   const Eigen::Quaterniond& _obj_orientation,
                                   const Eigen::MatrixXd& _obj_covariance,
                                   const Eigen::Vector3d& _odom_position,
                                   const Eigen::Quaterniond _odom_orientation,
                                   const Eigen::MatrixXd _odom_covariance) {
  // addNewObjectKeyframe();
  std::cout << "**** ADDING NEW ODOM KEYFRAME FROM OBJECT DETECTION ****" << std::endl;

  Eigen::Vector3d translation_from_last_node = _odom_position - last_node_position_;
  std::cout << _odom_position << last_node_position_ << std::endl;
  std::cout << "Tfln: " << translation_from_last_node << std::endl;
  Eigen::Quaterniond rotation_from_last_node = _odom_orientation * last_node_orientation_.inverse();
  rotation_from_last_node.normalize();
  std::cout << "Rfln: " << rotation_from_last_node << std::endl;

  addNewKeyframe(translation_from_last_node, rotation_from_last_node, _odom_covariance);

  std::cout << "**** ADDED NEW OBJECT KEYFRAME ****" << std::endl;
  // Change Identity to Odom pose?
  // Eigen::Isometry3d node_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d node_pose =
      Eigen::Translation3d(absolute_odom_position) * absolute_odom_orientation;

  auto [node, id] = addSE3Node(node_pose, graph_ptr_);
  std::cout << "ADDED NEW NODE" << std::endl;
  // Eigen::Isometry3d relative_pose;
  // relative_pose.setIdentity();
  // relative_pose.translation() = relative_position;
  Eigen::Isometry3d relative_pose = Eigen::Translation3d(_obj_position) * _obj_orientation;
  // auto last_node                  = graph_ptr_->vertices()[1];
  // auto prev_node                  = dynamic_cast<g2o::VertexSE3*>(last_node);
  //
  // auto prev_node = last_node_;
  // Eigen::MatrixXd::Identity(6, 6);
  addSE3Edge(last_node_, node, relative_pose, _obj_covariance, graph_ptr_);
  std::cout << "ADDED NEW EDGE" << std::endl;
  // last_node_ = node;
};

std::vector<std::array<double, 7>> OptimizerG2O::getGraph() {
  std::vector<std::array<double, 7>> pose_graph;
  // Get number of nodes of the graph
  // n_vertices_;
  pose_graph.reserve(n_vertices_);

  for (int i = 0; i < n_vertices_; i++) {
    graph_ptr_->vertex(i);
    // for (auto p : graph_ptr_->vertices()) {
    for (std::pair<const int, g2o::HyperGraph::Vertex*> p : graph_ptr_->vertices()) {
      int id = p.first;
      if (id != i) continue;

      auto node = dynamic_cast<g2o::VertexSE3*>(p.second);
      if (node) {
        auto T               = node->estimate().translation();
        Eigen::Quaterniond R = Eigen::Quaterniond(node->estimate().rotation());

        pose_graph.emplace_back(
            std::array<double, 7>{T.x(), T.y(), T.z(), R.w(), R.x(), R.y(), R.z()});
      }
      // else { Node is not VertexSE3 }
    }
  }
  return pose_graph;
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
