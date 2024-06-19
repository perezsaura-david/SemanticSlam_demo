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

#include "as2_core/names/topics.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"

#include <array>
#include <memory>
// #include <filesystem>
// #include <pluginlib/class_loader.hpp>
// #include "plugin_base.hpp"

SemanticSlam::SemanticSlam() : as2::Node("semantic_slam") {
  std::string default_odom_topic       = "cf0/sensor_measurements/noisy_odom";
  std::string default_aruco_pose_topic = "cf0/detect_aruco_markers_behavior/aruco_pose";
  std::string default_reference_frame  = "cf0/odom";
  std::string default_robot_frame      = "cf0/base_link";

  std::string default_viz_main_markers_topic = "slam_viz/markers/main";
  std::string default_viz_temp_markers_topic = "slam_viz/markers/temp";

  std::string default_viz_main_graph_topic      = "cf0/semantic_slam/graph";
  std::string default_viz_main_odom_nodes_topic = "slam_viz/main/odom_nodes";
  std::string default_viz_temp_odom_nodes_topic = "slam_viz/temp/odom_nodes";
  std::string default_viz_main_obj_nodes_topic  = "slam_viz/main/obj_nodes";
  std::string default_viz_temp_obj_nodes_topic  = "slam_viz/temp/obj_nodes";
  std::string default_viz_main_edges_topic      = "slam_viz/main/edges";
  std::string default_viz_temp_edges_topic      = "slam_viz/temp/edges";

  // std::shared_ptr<const rclcpp::QoS> reliable_qos =
  //     std::make_shared<const rclcpp::QoS>(rclcpp::QoS(2));
  // std::shared_ptr<const rclcpp::QoS> sensor_qos =
  //     std::make_shared<const rclcpp::QoS>(rclcpp::SensorDataQoS());
  rclcpp::QoS reliable_qos = rclcpp::QoS(10);
  rclcpp::QoS sensor_qos   = rclcpp::SensorDataQoS();

  // PARAMETERS
  std::string odom_topic = this->declare_parameter("odom_topic", default_odom_topic);
  std::string aruco_pose_topic =
      this->declare_parameter("aruco_pose_topic", default_aruco_pose_topic);
  reference_frame_ =
      this->declare_parameter<std::string>("reference_frame", default_reference_frame);
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", default_robot_frame);
  // VISUALIZATION
  std::string viz_main_markers_topic =
      this->declare_parameter("viz_main_markers_topic", default_viz_main_markers_topic);
  std::string viz_temp_markers_topic =
      this->declare_parameter("viz_temp_markers_topic", default_viz_temp_markers_topic);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, sensor_qos, std::bind(&SemanticSlam::odomCallback, this, std::placeholders::_1));
  aruco_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
      aruco_pose_topic, sensor_qos,
      std::bind(&SemanticSlam::arucoPoseCallback, this, std::placeholders::_1));

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  viz_main_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_main_markers_topic, reliable_qos);
  viz_temp_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      viz_temp_markers_topic, reliable_qos);

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

  // TODO: Define how to use this
  // msg->header.frame_id;
  // msg->header.stamp;

  // TODO: handle covariance
  bool new_node_added =
      optimizer_ptr_->handleNewOdom(odom_position, odom_orientation, odom_covariance);

  if (new_node_added) {
    visualizeMainGraph();
    visualizeCleanTempGraph();
  }
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateNodesMsg(
    std::shared_ptr<GraphG2O>& _graph,
    const std::string& _namespace,
    const std::string& _mode,
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
    node_marker_msg.ns   = _namespace;
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
    const std::string& _namespace,
    const std::array<float, 3>& _color) {
  std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>>> edge_lines =
      optimizer_ptr_->getEdgesLines(_graph);

  visualization_msgs::msg::MarkerArray edges_markers_msg;
  int counter = 0;
  for (auto edge_line : edge_lines) {
    visualization_msgs::msg::Marker edge_marker_msg;
    edge_marker_msg.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_marker_msg.header.frame_id = reference_frame_;
    edge_marker_msg.ns              = _namespace;
    // edge_marker_msg.id              = 0;
    edge_marker_msg.id = counter;
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

void SemanticSlam::getPoseFromMsg(const std::shared_ptr<as2_msgs::msg::PoseStampedWithID>& _msg,
                                  Eigen::Vector3d& _position,
                                  Eigen::Quaterniond& _orientation) {
  // Eigen::Vector3d aruco_position;
  // Eigen::Quaterniond aruco_orientation;
  std::string ref_frame = _msg->pose.header.frame_id;
  auto target_ts        = tf2::TimePointZero;
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
      tf2::doTransform(_msg->pose, transformed_pose, ref_frame_transform);
      _position.x()    = transformed_pose.pose.position.x;
      _position.y()    = transformed_pose.pose.position.y;
      _position.z()    = transformed_pose.pose.position.z;
      _orientation.w() = transformed_pose.pose.orientation.w;
      _orientation.x() = transformed_pose.pose.orientation.x;
      _orientation.y() = transformed_pose.pose.orientation.y;
      _orientation.z() = transformed_pose.pose.orientation.z;

    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", ref_frame.c_str(),
                  reference_frame_.c_str(), ex.what());
      return;
    }
  } else {
    _position.x()    = _msg->pose.pose.position.x;
    _position.y()    = _msg->pose.pose.position.y;
    _position.z()    = _msg->pose.pose.position.z;
    _orientation.w() = _msg->pose.pose.orientation.w;
    _orientation.x() = _msg->pose.pose.orientation.x;
    _orientation.y() = _msg->pose.pose.orientation.y;
    _orientation.z() = _msg->pose.pose.orientation.z;
  }
}

void SemanticSlam::arucoPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Aruco received: '%s'", msg->id.c_str());
  std::string aruco_id = msg->id;
  Eigen::Vector3d aruco_position;
  Eigen::Quaterniond aruco_orientation;

  // TODO: Define how to use this
  // msg->pose.header.frame_id;
  // msg->pose.header.stamp;

  getPoseFromMsg(msg, aruco_position, aruco_orientation);
  // Covariance
  Eigen::Matrix<double, 6, 6> aruco_covariance = Eigen::MatrixXd::Identity(6, 6) * 10.0;
  // Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> odom_covariance(
  //     msg->pose.covariance.data());

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
  visualization_msgs::msg::MarkerArray viz_odom_nodes_msg =
      generateNodesMsg(optimizer_ptr_->main_graph, "main/odom", "odom", odom_node_color);
  viz_main_markers_pub_->publish(viz_odom_nodes_msg);

  visualization_msgs::msg::MarkerArray viz_obj_nodes_msg =
      generateNodesMsg(optimizer_ptr_->main_graph, "main/obj", "obj", obj_node_color);
  viz_main_markers_pub_->publish(viz_obj_nodes_msg);

  visualization_msgs::msg::MarkerArray viz_edges_msg =
      generateEdgesMsg(optimizer_ptr_->main_graph, "main/edges", edge_color);
  viz_main_markers_pub_->publish(viz_edges_msg);
}

void SemanticSlam::visualizeTempGraph() {
  // VISUALIZATION MSGS
  std::array<float, 3> odom_node_color{0.0, 1.0, 0.0};
  std::array<float, 3> obj_node_color{1.0, 1.0, 0.5};
  std::array<float, 3> edge_color{5.0, 1.0, 0.5};

  visualization_msgs::msg::MarkerArray viz_odom_nodes_msg =
      generateNodesMsg(optimizer_ptr_->temp_graph, "temp/odom", "odom", odom_node_color);
  viz_temp_markers_pub_->publish(viz_odom_nodes_msg);

  visualization_msgs::msg::MarkerArray viz_obj_nodes_msg =
      generateNodesMsg(optimizer_ptr_->temp_graph, "temp/obj", "obj", obj_node_color);
  viz_temp_markers_pub_->publish(viz_obj_nodes_msg);

  visualization_msgs::msg::MarkerArray viz_edges_msg =
      generateEdgesMsg(optimizer_ptr_->temp_graph, "temp/edges", edge_color);
  viz_temp_markers_pub_->publish(viz_edges_msg);
}

void SemanticSlam::visualizeCleanTempGraph() {
  // WARN("Cleaning Temp Graph Visualization");
  visualization_msgs::msg::MarkerArray viz_clean_markers_msg = generateCleanMarkersMsg("temp/odom");
  viz_temp_markers_pub_->publish(viz_clean_markers_msg);
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateCleanMarkersMsg(
    const std::string& _namespace) {
  visualization_msgs::msg::MarkerArray markers_msg;
  visualization_msgs::msg::Marker marker_msg;
  // marker_msg.ns     = _namespace;
  marker_msg.id     = 0;
  marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;
  markers_msg.markers.emplace_back(marker_msg);
  return markers_msg;
}

// nav_msgs::msg::Path SemanticSlam::generateGraphMsg(
//     const std::vector<std::array<double, 7>>& _graph) {
//   nav_msgs::msg::Path graph_msg;
//   // FIXME: Get the frame id
//   graph_msg.header.frame_id = reference_frame_;

//   for (auto pose : _graph) {
//     geometry_msgs::msg::PoseStamped pose_msg;
//     pose_msg.pose.position.x    = pose[0];
//     pose_msg.pose.position.y    = pose[1];
//     pose_msg.pose.position.z    = pose[2];
//     pose_msg.pose.orientation.w = pose[3];
//     pose_msg.pose.orientation.x = pose[4];
//     pose_msg.pose.orientation.y = pose[5];
//     pose_msg.pose.orientation.z = pose[6];

//     graph_msg.poses.emplace_back(pose_msg);
//   }

//   return graph_msg;
// }
