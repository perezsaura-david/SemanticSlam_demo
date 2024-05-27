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
  std::string default_reference_frame  = "cf0/odom";
  std::string default_robot_frame      = "cf0/base_link";

  std::string default_viz_main_graph_topic      = "cf0/semantic_slam/graph";
  std::string default_viz_main_odom_nodes_topic = "slam_viz/main/odom_nodes";
  std::string default_viz_temp_odom_nodes_topic = "slam_viz/temp/odom_nodes";
  std::string default_viz_main_obj_nodes_topic  = "slam_viz/main/obj_nodes";
  std::string default_viz_temp_obj_nodes_topic  = "slam_viz/temp/obj_nodes";
  std::string default_viz_main_edges_topic      = "slam_viz/main/edges";
  std::string default_viz_temp_edges_topic      = "slam_viz/temp/edges";

  std::shared_ptr<const rclcpp::QoS> reliable_qos =
      std::make_shared<const rclcpp::QoS>(rclcpp::QoS(2));
  std::shared_ptr<const rclcpp::QoS> sensor_qos =
      std::make_shared<const rclcpp::QoS>(rclcpp::SensorDataQoS());

  // PARAMETERS
  std::string odom_topic = this->declare_parameter("odom_topic", default_odom_topic);
  std::string aruco_pose_topic =
      this->declare_parameter("aruco_pose_topic", default_aruco_pose_topic);
  reference_frame_ =
      this->declare_parameter<std::string>("reference_frame", default_reference_frame);
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", default_robot_frame);
  // VISUALIZATION
  std::string viz_main_graph_topic =
      this->declare_parameter("graph_topic", default_viz_main_graph_topic);
  std::string viz_main_odom_nodes_topic =
      this->declare_parameter("viz_main_odom_nodes_topic", default_viz_main_odom_nodes_topic);
  std::string viz_temp_odom_nodes_topic =
      this->declare_parameter("viz_temp_odom_nodes_topic", default_viz_temp_odom_nodes_topic);
  std::string viz_main_obj_nodes_topic =
      this->declare_parameter("viz_main_obj_nodes_topic", default_viz_main_obj_nodes_topic);
  std::string viz_temp_obj_nodes_topic =
      this->declare_parameter("viz_temp_obj_nodes_topic", default_viz_temp_obj_nodes_topic);
  std::string viz_main_edges_topic =
      this->declare_parameter("viz_main_edges_topic", default_viz_main_edges_topic);
  std::string viz_temp_edges_topic =
      this->declare_parameter("viz_temp_edges_topic", default_viz_temp_edges_topic);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, *sensor_qos, std::bind(&SemanticSlam::odomCallback, this, std::placeholders::_1));
  aruco_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
      aruco_pose_topic, *sensor_qos,
      std::bind(&SemanticSlam::arucoPoseCallback, this, std::placeholders::_1));

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  viz_main_graph_pub_ =
      this->create_publisher<nav_msgs::msg::Path>(viz_main_graph_topic, *reliable_qos);
  viz_main_odom_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_main_odom_nodes_topic, *reliable_qos);
  viz_temp_odom_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_temp_odom_nodes_topic, *reliable_qos);
  viz_main_obj_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_main_obj_nodes_topic, *reliable_qos);
  viz_temp_obj_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_temp_obj_nodes_topic, *reliable_qos);
  viz_main_edges_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_main_edges_topic, *reliable_qos);
  viz_temp_edges_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_temp_edges_topic, *reliable_qos);

  optimizer_ptr_ = std::make_unique<OptimizerG2O>();
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
  bool new_node_added =
      optimizer_ptr_->handleNewOdom(odom_position, odom_orientation, odom_covariance);

  // VISUALIZATION MSGS
  // std::vector<std::array<double, 7>> graph = optimizer_ptr_->main_graph->getGraph();
  // nav_msgs::msg::Path graph_msg            = generateGraphMsg(graph);
  // graph_pub_->publish(graph_msg);
  // visualization_msgs::msg::MarkerArray odom_nodes_msg = generateOdomNodesMsg();
  // odom_nodes_pub_->publish(odom_nodes_msg);
  //
  if (new_node_added) {
    visualizeMainGraph();
    visualizeCleanTempGraph();
  }
}

// visualization_msgs::msg::MarkerArray SemanticSlam::generateOdomNodesMsg() {
//   std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes_poses =
//       optimizer_ptr_->getOdomNodePoses();
//   visualization_msgs::msg::MarkerArray nodes_markers_msg;
//   // std::cout << "REFERENCE FRAME :" << reference_frame_ << std::endl;
//   // nodes_markers_msg.header.frame_id = reference_frame_;
//   int counter = 0;
//   for (auto node_pose : odom_nodes_poses) {
//     visualization_msgs::msg::Marker node_marker_msg;
//     node_marker_msg.type = node_marker_msg.ARROW;
//     node_marker_msg.id   = counter;
//     counter++;
//     node_marker_msg.header.frame_id    = reference_frame_;
//     node_marker_msg.pose.position.x    = node_pose.first.x();
//     node_marker_msg.pose.position.y    = node_pose.first.y();
//     node_marker_msg.pose.position.z    = node_pose.first.z();
//     node_marker_msg.pose.orientation.w = node_pose.second.w();
//     node_marker_msg.pose.orientation.x = node_pose.second.x();
//     node_marker_msg.pose.orientation.y = node_pose.second.y();
//     node_marker_msg.pose.orientation.z = node_pose.second.z();
//     node_marker_msg.scale.x            = 0.5;
//     node_marker_msg.scale.y            = 0.05;
//     node_marker_msg.scale.z            = 0.05;
//     node_marker_msg.color.b            = 1.0;
//     node_marker_msg.color.a            = 1.0;
//     nodes_markers_msg.markers.emplace_back(node_marker_msg);
//   }
//   return nodes_markers_msg;
// }

// visualization_msgs::msg::MarkerArray SemanticSlam::generateObjOdomNodesMsg() {
//   std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes_poses =
//       optimizer_ptr_->getObjNodePoses("odom");
//   visualization_msgs::msg::MarkerArray nodes_markers_msg;
//   // std::cout << "REFERENCE FRAME :" << reference_frame_ << std::endl;
//   // nodes_markers_msg.header.frame_id = reference_frame_;
//   int counter = 0;
//   for (auto node_pose : odom_nodes_poses) {
//     visualization_msgs::msg::Marker node_marker_msg;
//     node_marker_msg.type = node_marker_msg.ARROW;
//     node_marker_msg.id   = counter;
//     counter++;
//     node_marker_msg.header.frame_id    = reference_frame_;
//     node_marker_msg.pose.position.x    = node_pose.first.x();
//     node_marker_msg.pose.position.y    = node_pose.first.y();
//     node_marker_msg.pose.position.z    = node_pose.first.z();
//     node_marker_msg.pose.orientation.w = node_pose.second.w();
//     node_marker_msg.pose.orientation.x = node_pose.second.x();
//     node_marker_msg.pose.orientation.y = node_pose.second.y();
//     node_marker_msg.pose.orientation.z = node_pose.second.z();
//     node_marker_msg.scale.x            = 0.5;
//     node_marker_msg.scale.y            = 0.05;
//     node_marker_msg.scale.z            = 0.05;
//     node_marker_msg.color.g            = 1.0;
//     node_marker_msg.color.a            = 1.0;
//     nodes_markers_msg.markers.emplace_back(node_marker_msg);
//   }
//   return nodes_markers_msg;
// }

// visualization_msgs::msg::MarkerArray SemanticSlam::generateObjNodesMsg() {
//   std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes_poses =
//       optimizer_ptr_->getNodePoses(optimizer_ptr_->temp_graph, "obj");
//   visualization_msgs::msg::MarkerArray nodes_markers_msg;
//   // std::cout << "REFERENCE FRAME :" << reference_frame_ << std::endl;
//   // nodes_markers_msg.header.frame_id = reference_frame_;
//   int counter = 0;
//   for (auto node_pose : odom_nodes_poses) {
//     visualization_msgs::msg::Marker node_marker_msg;
//     node_marker_msg.type = node_marker_msg.ARROW;
//     node_marker_msg.id   = counter;
//     counter++;
//     node_marker_msg.header.frame_id    = reference_frame_;
//     node_marker_msg.pose.position.x    = node_pose.first.x();
//     node_marker_msg.pose.position.y    = node_pose.first.y();
//     node_marker_msg.pose.position.z    = node_pose.first.z();
//     node_marker_msg.pose.orientation.w = node_pose.second.w();
//     node_marker_msg.pose.orientation.x = node_pose.second.x();
//     node_marker_msg.pose.orientation.y = node_pose.second.y();
//     node_marker_msg.pose.orientation.z = node_pose.second.z();
//     node_marker_msg.scale.x            = 0.5;
//     node_marker_msg.scale.y            = 0.05;
//     node_marker_msg.scale.z            = 0.05;
//     node_marker_msg.color.r            = 1.0;
//     node_marker_msg.color.a            = 1.0;
//     nodes_markers_msg.markers.emplace_back(node_marker_msg);
//   }
//   return nodes_markers_msg;
// }

visualization_msgs::msg::MarkerArray SemanticSlam::generateNodesMsg(
    std::shared_ptr<GraphG2O>& _graph,
    std::string _mode,
    const std::array<float, 3>& _color) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes_poses =
      optimizer_ptr_->getNodePoses(_graph, _mode);
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
    node_marker_msg.color.r            = _color[0];
    node_marker_msg.color.g            = _color[1];
    node_marker_msg.color.b            = _color[2];
    node_marker_msg.color.a            = 1.0;
    nodes_markers_msg.markers.emplace_back(node_marker_msg);
  }
  return nodes_markers_msg;
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateEdgesMsg(
    std::shared_ptr<GraphG2O>& _graph,
    const std::array<float, 3>& _color) {
  std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>>> edge_lines =
      optimizer_ptr_->getEdgesLines(_graph);

  visualization_msgs::msg::MarkerArray edges_markers_msg;
  int counter = 0;
  for (auto edge_line : edge_lines) {
    visualization_msgs::msg::Marker edge_marker_msg;
    edge_marker_msg.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_marker_msg.header.frame_id = reference_frame_;
    edge_marker_msg.id              = 0;
    edge_marker_msg.id              = counter;
    counter++;
    // edge_marker_msg.action          = visualization_msgs::msg::Marker::ADD;
    // Set the scale of the line (thickness)
    edge_marker_msg.scale.x = 0.01;
    // Set the color (r, g, b, a)
    edge_marker_msg.color.r = _color[0];
    edge_marker_msg.color.g = _color[1];
    edge_marker_msg.color.b = _color[2];
    edge_marker_msg.color.a = 1.0;

    // Define the points in the line
    geometry_msgs::msg::Point p0;
    p0.x = edge_line[0].first.x();
    p0.y = edge_line[0].first.y();
    p0.z = edge_line[0].first.z();
    geometry_msgs::msg::Point p1;
    p1.x = edge_line[1].first.x();
    p1.y = edge_line[1].first.y();
    p1.z = edge_line[1].first.z();
    edge_marker_msg.points.emplace_back(p0);
    edge_marker_msg.points.emplace_back(p1);

    visualization_msgs::msg::MarkerArray marker_array;
    edges_markers_msg.markers.emplace_back(edge_marker_msg);
  }
  return edges_markers_msg;
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
  // static int counter = 0;
  // counter += 1;
  // if (counter < 10) {
  //   return;  // NO ARUCO DETECTION
  // }
  // counter = 0;

  // DEBUG
  // std::cout << "P__: " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << "
  // "
  //           << msg->pose.pose.position.z << std::endl;
  // std::cout << "O__: " << msg->pose.pose.orientation.w << " " << msg->pose.pose.orientation.x
  // << " "
  //           << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z <<
  //           std::endl;

  aruco_id              = msg->id;
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
      // std::cout << "Ref frame changed to: " << ref_frame << std::endl;
    }
  }

  if (ref_frame != reference_frame_) {
    geometry_msgs::msg::TransformStamped ref_frame_transform;
    try {
      // TODO: Make a function for this
      // std::cout << "Transformation between " << reference_frame_ << " and " << ref_frame
      // << std::endl;
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

  // std::cout << "PPP: " << aruco_position << std::endl;
  // std::cout << "OOO: " << aruco_orientation << std::endl;

  // Covariance
  // Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> odom_covariance(
  //     msg->pose.covariance.data());

  // TODO: Define how to use this
  // msg->pose.header.frame_id;
  // msg->pose.header.stamp;
  // std::cout << "ARUCO: add last odom" << std::endl;
  // Eigen::Matrix<double, 6, 6>(),
  Eigen::Matrix<double, 6, 6> aruco_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;

  // std::cout << "ARUCO: object detected" << std::endl;
  optimizer_ptr_->handleNewObject(aruco_id, aruco_position, aruco_orientation, aruco_covariance,
                                  last_odom_abs_position_received_,
                                  last_odom_abs_orientation_received_,
                                  last_odom_abs_covariance_received_);
  visualizeTempGraph();
}

void SemanticSlam::visualizeMainGraph() {
  // VISUALIZATION MSGS
  std::array<float, 3> odom_node_color{0.0, 0.0, 1.0};
  std::array<float, 3> obj_node_color{1.0, 0.0, 0.0};
  std::array<float, 3> edge_color{0.5, 0.5, 1.0};
  visualization_msgs::msg::MarkerArray odom_nodes_msg =
      generateNodesMsg(optimizer_ptr_->main_graph, "odom", odom_node_color);
  viz_main_odom_nodes_pub_->publish(odom_nodes_msg);
  visualization_msgs::msg::MarkerArray obj_nodes_msg =
      generateNodesMsg(optimizer_ptr_->main_graph, "obj", obj_node_color);
  viz_main_obj_nodes_pub_->publish(obj_nodes_msg);
  visualization_msgs::msg::MarkerArray viz_main_edges_msg =
      generateEdgesMsg(optimizer_ptr_->main_graph, edge_color);
  viz_main_edges_pub_->publish(viz_main_edges_msg);
}

void SemanticSlam::visualizeTempGraph() {
  // VISUALIZATION MSGS
  std::array<float, 3> odom_node_color{0.0, 1.0, 0.0};
  std::array<float, 3> obj_node_color{1.0, 1.0, 0.5};
  std::array<float, 3> edge_color{5.0, 1.0, 0.5};
  visualization_msgs::msg::MarkerArray viz_odom_nodes_msg =
      generateNodesMsg(optimizer_ptr_->temp_graph, "odom", odom_node_color);
  // INFO("Viz Temp Odom: " << viz_odom_nodes_msg.markers.size());
  viz_temp_odom_nodes_pub_->publish(viz_odom_nodes_msg);

  visualization_msgs::msg::MarkerArray viz_obj_nodes_msg =
      generateNodesMsg(optimizer_ptr_->temp_graph, "obj", obj_node_color);
  // INFO("Viz Temp Objects: " << viz_obj_nodes_msg.markers.size());
  viz_temp_obj_nodes_pub_->publish(viz_obj_nodes_msg);

  visualization_msgs::msg::MarkerArray viz_temp_edges_msg =
      generateEdgesMsg(optimizer_ptr_->temp_graph, edge_color);
  // INFO("Viz Temp Edges: " << viz_temp_edges_msg.markers.size());
  viz_temp_edges_pub_->publish(viz_temp_edges_msg);
}

void SemanticSlam::visualizeCleanTempGraph() {
  WARN("Cleaning Temp Graph Visualization");
  visualization_msgs::msg::MarkerArray viz_clean_markers_msg = generateCleanMarkersMsg();
  viz_temp_odom_nodes_pub_->publish(viz_clean_markers_msg);
  viz_temp_obj_nodes_pub_->publish(viz_clean_markers_msg);
  viz_temp_edges_pub_->publish(viz_clean_markers_msg);
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateCleanMarkersMsg() {
  visualization_msgs::msg::MarkerArray markers_msg;
  visualization_msgs::msg::Marker marker_msg;
  // node_marker_msg.type               = node_marker_msg.ARROW;
  marker_msg.id     = 0;
  marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;
  markers_msg.markers.emplace_back(marker_msg);
  return markers_msg;
}

//*********************************************************************************************//
//*********************************************************************************************//
//*********************************************************************************************//

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

OptimizerG2O::OptimizerG2O() {
  FLAG("STARTING SEMANTIC SLAM");

  // FLAG("Create Main Graph");
  main_graph = std::make_shared<GraphG2O>("Main graph");
  // FLAG("Create Temp Graph");
  temp_graph = std::make_shared<GraphG2O>("Temp graph");

  if (odometry_is_relative_) {
    WARN("Relative odometry");
  } else {
    WARN("Absolute odometry");
  }

  main_graph->initGraph();
}

// std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> OptimizerG2O::getOdomNodePoses() {
//   std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> odom_nodes;
//   // std::cout << "ODOM_NODES_SIZE: " << odom_nodes_.size() << std::endl;
//   odom_nodes.reserve(main_graph->odom_nodes_.size());

//   for (auto node : main_graph->odom_nodes_) {
//     std::pair<Eigen::Vector3d, Eigen::Quaterniond> node_pose;
//     if (getNodePose(node, node_pose)) {
//       odom_nodes.emplace_back(node_pose);
//     } else
//       ERROR("CAN'T CONVERT VERTEX TO SE3");
//   }
//   return odom_nodes;
// }

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

    // std::cout << node_poses[0].first.transpose() << " - " << node_poses[1].first.transpose()
    //           << std::endl;

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

  // TODO: Check to do this inside the next function
  // main_graph->last_node_pose_ = absolute_odom_pose;

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

//*********************************************************************************************//
//*********************************************************************************************//
//*********************************************************************************************//

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
  int iterations = graph_->optimize(num_iterations);
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
  // Check Information Matrix
  // DEBUG("information_matrix");
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

  addSE3Edge(last_node_, node, _relative_pose, _relative_covariance);
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
  Eigen::Isometry3d node_pose = Eigen::Isometry3d::Identity();

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

  addSE3Edge(last_node_, object_node, _obj_relative_pose, _obj_covariance);
  // INFO("Added new edge to object");
}

std::vector<ObjectNodeInfo> GraphG2O::getObjectNodes() { return obj_nodes_info_; }

ObjectNodeInfo::ObjectNodeInfo(const std::string _id,
                               g2o::HyperGraph::Vertex* _node,
                               // g2o::VertexSE3* _node,
                               const Eigen::MatrixXd& _covariance) {
  object_id  = _id;
  node       = _node;
  covariance = _covariance;
}
