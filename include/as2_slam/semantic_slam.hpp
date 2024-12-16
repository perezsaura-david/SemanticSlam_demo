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


/********************************************************************************************
 *  \file       semantic_slam.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
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

#include "optimizer_g2o.hpp"
#include "utils/conversions.hpp"

// ROS2
#include <Eigen/src/Geometry/Transform.h>
#include <as2_core/node.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS2 MSGS
#include <as2_msgs/msg/pose_stamped_with_id.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>

class SemanticSlam : public as2::Node
{
public:
  SemanticSlam();
  ~SemanticSlam() {}
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void arucoPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg);

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr aruco_pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_main_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_temp_markers_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_frame_;
  bool odometry_is_relative_ = false;

  std::unique_ptr<OptimizerG2O> optimizer_ptr_;  // g2o graph

  // PoseSE3 last_odom_abs_pose_received_;
  Eigen::Isometry3d last_odom_abs_pose_received_;
  Eigen::MatrixXd last_odom_abs_covariance_received_;
  geometry_msgs::msg::TransformStamped map_odom_transform_msg_;

  Eigen::Isometry3d generatePoseFromMsg(
    const std::shared_ptr<as2_msgs::msg::PoseStampedWithID> & _msg);

  void visualizeCleanTempGraph();
  void visualizeMainGraph();
  void visualizeTempGraph();

  visualization_msgs::msg::MarkerArray generateVizNodesMsg(std::shared_ptr<GraphG2O> & _graph);
  visualization_msgs::msg::MarkerArray generateVizEdgesMsg(std::shared_ptr<GraphG2O> & _graph);
  visualization_msgs::msg::MarkerArray generateCleanMarkersMsg();
  void updateMapOdomTransform(const std_msgs::msg::Header & _header);

  // tf_broadcaster_; std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
  // tfstatic_broadcaster_; std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  // std::filesystem::path plugin_name_;
  // std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>
  //     loader_;
  // std::shared_ptr<as2_state_estimator_plugin_base::StateEstimatorBase>
  // plugin_ptr_; std::shared_ptr<tf2_ros::TransformBroadcaster>
};

#endif  // __AS2__SEMANTIC_SLAM_HPP_
