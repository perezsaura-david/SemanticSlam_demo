/********************************************************************************************
 *  \file       semantic_slam.hpp
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

#ifndef __AS2__SEMANTIC_SLAM_HPP_
#define __AS2__SEMANTIC_SLAM_HPP_

#include <as2_msgs/msg/detail/pose_stamped_with_id__struct.hpp>
#define RESET_COLOR "\033[0m"
#define RED_COLOR "\033[0;31m"
#define GREEN_COLOR "\033[1;32m"
#define YELLOW_COLOR "\033[1;33m"
#define MAGENTA_COLOR "\033[0;35m"
#define CYAN_COLOR "\033[0;36m"
#define ERROR(x)                                                                        \
  std::cerr << RED_COLOR << "[ERROR] " << x << std::endl                                \
            << "\t  at line " << __LINE__ << " in function " << __func__ << RESET_COLOR \
            << std::endl

#define WARN(x) std::cout << YELLOW_COLOR << "[WARN] " << x << RESET_COLOR << std::endl

#define FLAG(x) std::cout << CYAN_COLOR << "[FLAG] " << x << RESET_COLOR << std::endl
#define DEBUG(x)                                                                        \
  std::cout << YELLOW_COLOR << "[DEBUG] " << x << std::endl                             \
            << "\t  at line " << __LINE__ << " in function " << __func__ << RESET_COLOR \
            << std::endl
#define INFO(x) std::cout << x << std::endl;

// ROS2
#include <array>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/tf_utils.hpp"

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

// #include <filesystem>
// #include <pluginlib/class_loader.hpp>
// #include "plugin_base.hpp"

// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Backend
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/eigen_types.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

class ObjectNodeInfo {
public:
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
  std::map<std::string, int> obj_id2node_;
  std::vector<ObjectNodeInfo> obj_nodes_info_;

private:
  int n_vertices_ = 0;
  int n_edges_    = 0;
  std::string name_;
};

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
  double obj_odometry_distance_threshold_    = 0.2;  // meters
  double obj_odometry_orientation_threshold_ = 0.5;  // radians
  bool odometry_is_relative_                 = false;
};

class SemanticSlam : public as2::Node {
public:
  SemanticSlam();
  ~SemanticSlam(){};
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void arucoPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg);

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr aruco_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_main_graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_main_odom_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_temp_odom_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_main_obj_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_temp_obj_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_main_edges_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_temp_edges_pub_;
  std::string reference_frame_;
  std::string robot_frame_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::filesystem::path plugin_name_;
  // std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>
  //     loader_;
  // std::shared_ptr<as2_state_estimator_plugin_base::StateEstimatorBase>
  // plugin_ptr_; std::shared_ptr<tf2_ros::TransformBroadcaster>
  // tf_broadcaster_; std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
  // tfstatic_broadcaster_; std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  std::unique_ptr<OptimizerG2O> optimizer_ptr_;  // g2o graph
  Eigen::Vector3d last_odom_abs_position_received_;
  Eigen::Quaterniond last_odom_abs_orientation_received_;
  Eigen::MatrixXd last_odom_abs_covariance_received_;

  // void getPoseFromMsg(const as2_msgs::msg::PoseStampedWithID& _msg,
  void getPoseFromMsg(const std::shared_ptr<as2_msgs::msg::PoseStampedWithID>& _msg,
                      Eigen::Vector3d& _position,
                      Eigen::Quaterniond& _orientation);
  nav_msgs::msg::Path generateGraphMsg(const std::vector<std::array<double, 7>>& _graph);
  visualization_msgs::msg::MarkerArray generateEdgesMsg(std::shared_ptr<GraphG2O>& _graph,
                                                        const std::array<float, 3>& color);
  visualization_msgs::msg::MarkerArray generateNodesMsg(std::shared_ptr<GraphG2O>& _graph,
                                                        std::string _mode,
                                                        const std::array<float, 3>& _color);
  visualization_msgs::msg::MarkerArray generateCleanMarkersMsg();
  void visualizeCleanTempGraph();
  void visualizeMainGraph();
  void visualizeTempGraph();
};

#endif  // __AS2__SEMANTIC_SLAM_HPP_
