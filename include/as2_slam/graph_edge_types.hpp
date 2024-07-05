/********************************************************************************************
 *  \file       graph_edge_types.hpp
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

#ifndef __AS2__GRAPH_EDGE_TYPES_HPP_
#define __AS2__GRAPH_EDGE_TYPES_HPP_

#include <Eigen/Dense>
#include <g2o/core/hyper_graph.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <string>

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "as2_slam/graph_node_types.hpp"
#include "utils/general_utils.hpp"


class GraphEdge
{
public:
  virtual g2o::HyperGraph::Edge * getEdge()               = 0;
  virtual visualization_msgs::msg::Marker getVizMarker() = 0;
  virtual Eigen::Vector4d getVizMarkerColor()            = 0;
  virtual std::string getVizMarkerNamespace()            = 0;
  virtual std::string getEdgeName()                      = 0;
};

class GraphEdgeSE3 : public GraphEdge
{
public:
  GraphEdgeSE3(
    GraphNodeSE3 * _node1,
    GraphNodeSE3 * _node2,
    const Eigen::Isometry3d & _relative_pose,
    const Eigen::MatrixXd & _information_matrix)
  {
    edge_ = new g2o::EdgeSE3();
    edge_->setMeasurement(_relative_pose);
    edge_->setInformation(_information_matrix);
    edge_->vertices()[0] = _node1->getVertexSE3();
    edge_->vertices()[1] = _node2->getVertexSE3();
    if (_information_matrix.size() == 0) {
      WARN("Information Matrix Empty");
    }
  }
  ~GraphEdgeSE3() {}

  virtual g2o::HyperGraph::Edge * getEdge() override
  {
    return static_cast<g2o::HyperGraph::Edge *>(edge_);
  }

  g2o::EdgeSE3 * getEdgeSE3() {return edge_;}

  virtual visualization_msgs::msg::Marker getVizMarker() override
  {
    visualization_msgs::msg::Marker edge_marker_msg;
    edge_marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // node_marker_msg.header.frame_id = getReferenceFrame();
    edge_marker_msg.ns = getVizMarkerNamespace();
    edge_marker_msg.id = edge_->id();
    edge_marker_msg.scale.x = 0.01;  // Thickness
    Eigen::Vector4d color = getVizMarkerColor();
    edge_marker_msg.color.r = color[0];
    edge_marker_msg.color.g = color[1];
    edge_marker_msg.color.b = color[2];
    edge_marker_msg.color.a = color[3];
    // Define the points in the line
    for (int i = 0; i < 2; i++) {
      g2o::VertexSE3 * node_se3 = dynamic_cast<g2o::VertexSE3 *>(getEdge()->vertices()[i]);
      auto position = node_se3->estimate().translation();
      geometry_msgs::msg::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      edge_marker_msg.points.emplace_back(point);
    }
    return edge_marker_msg;
  }

protected:
  std::string getEdgeName() override {return edge_name_;}
  std::string getVizMarkerNamespace() override
  {
    return element_name_ + "/" + getEdgeName();
  }
  Eigen::Vector4d getVizMarkerColor() override {return viz_color_;}

  g2o::EdgeSE3 * edge_;
  std::string element_name_ = "edge";
  std::string edge_name_ = "SE3";
  Eigen::Vector4d viz_color_ = {1.0, 1.0, 1.0, 1.0};
};

class ArucoEdge : public GraphEdgeSE3
{
public:
  ArucoEdge(
    GraphNodeSE3 * _node1,
    GraphNodeSE3 * _node2,
    const Eigen::Isometry3d & _pose,
    const Eigen::MatrixXd & _information_matrix)
  : GraphEdgeSE3(_node1, _node2, _pose, _information_matrix)
  {
    edge_name_ = "Aruco";
    viz_color_ = {0.0, 1.0, 0.0, 1.0};
  }
};

class OdomEdge : public GraphEdgeSE3
{
public:
  OdomEdge(
    GraphNodeSE3 * _node1,
    GraphNodeSE3 * _node2,
    const Eigen::Isometry3d & _pose,
    const Eigen::MatrixXd & _information_matrix)
  : GraphEdgeSE3(_node1, _node2, _pose, _information_matrix) {}

protected:
  std::string edge_name_ = "Odometry";
  Eigen::Vector4d viz_color_ = {0.0, 0.0, 1.0, 1.0};
  virtual std::string getEdgeName() override {return edge_name_;}
  virtual Eigen::Vector4d getVizMarkerColor() override {return viz_color_;}
};

#endif  // __AS2__GRAPH_EDGE_TYPES_HPP_
