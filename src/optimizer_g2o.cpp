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
 * @file optimizer_g2o.cpp
 *
 * OptimizerG2O class implementation for AeroStack2
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "as2_slam/optimizer_g2o.hpp"
#include <Eigen/src/Geometry/Transform.h>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include "as2_slam/graph_g2o.hpp"
#include "as2_slam/graph_node_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"
// #include "utils/debug_utils.hpp"

OptimizerG2O::OptimizerG2O()
{
  FLAG("STARTING SEMANTIC SLAM");

  main_graph = std::make_shared<GraphG2O>("Main Graph");
  temp_graph = std::make_shared<GraphG2O>("Temp Graph");

  if (odometry_is_relative_) {
    WARN("Relative odometry");
  } else {
    WARN("Absolute odometry");
  }

  // TODO(dps): Make this a parameter
  std::vector<std::pair<std::string, Eigen::Vector3d>> fixed_objects_list = {
    {"gate_1", Eigen::Vector3d(4.0, 1.3, 1.13)},
    {"gate_2", Eigen::Vector3d(4.0, -1.34, 1.16)},
    {"gate_3", Eigen::Vector3d(-4.0, -1.29, 1.16)},
    {"gate_4", Eigen::Vector3d(-3.97, 1.28, 1.17)}
  };
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // Assuming default orientation

  std::vector<IsometryWithID> fixed_objects;
  for (auto object : fixed_objects_list) {
    IsometryWithID fixed_object;
    fixed_object.id = object.first;
    fixed_object.isometry = convertToIsometry3d(object.second, orientation);
    fixed_objects.emplace_back(fixed_object);
  }

  main_graph->initGraph();
  main_graph->setFixedObjects(fixed_objects);
  map_odom_tranform_ = Eigen::Isometry3d::Identity();
  last_abs_odom_pose_received_ = Eigen::Isometry3d::Identity();
  last_transformed_odom_received_ = Eigen::Isometry3d::Identity();
}

void OptimizerG2O::generateRelativeAndAbsoluteOdometryPoses(
  const Eigen::Isometry3d & _new_odometry_pose,
  Eigen::Isometry3d & _absolute_odom_pose,
  Eigen::Isometry3d & _relative_odom_pose)
{
  if (odometry_is_relative_) {
    // TODO(dps): RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
  } else {
    // ABSOLUTE ODOMETRY
    _absolute_odom_pose = _new_odometry_pose;
    _relative_odom_pose = last_abs_odom_pose_received_.inverse() * _absolute_odom_pose;
  }
}

bool OptimizerG2O::handleNewOdom(
  const Eigen::Isometry3d & _new_odometry_pose,
  const Eigen::MatrixXd & _odometry_covariance)
{
  Eigen::Isometry3d absolute_odom_pose;
  Eigen::Isometry3d relative_odom_pose;
  generateRelativeAndAbsoluteOdometryPoses(
    _new_odometry_pose, absolute_odom_pose,
    relative_odom_pose);
  Eigen::Isometry3d transformed_odom_pose = map_odom_tranform_ * absolute_odom_pose;

  // WARN("new_odom_pose");
  // auto pose = convertToPoseSE3(absolute_odom_pose);
  // INFO(pose.position.x() << " " << pose.position.y() << " " << pose.position.z());
  // pose = convertToPoseSE3(transformed_odom_pose);
  // INFO(pose.position.x() << " " << pose.position.y() << " " << pose.position.z());

  // TODO(dps): check time from the last odometry received
  // Check distance from the last odometry received
  double translation_distance_from_last_node = relative_odom_pose.translation().norm();
  // FIXME: get rotation distance
  // double rotation_distance_from_last_node = relative_odom_pose.rotation().norm();
  if (translation_distance_from_last_node < odometry_distance_threshold_) {
    // if (rotation_distance_from_last_node < odometry_orientation_threshold_) {
    // INFO("New odometry distance is not enough: " << translation_distance_from_last_node);
    return false;
    // }
  }
  // INFO("New odometry distance is enough: " << translation_distance_from_last_node);
  last_abs_odom_pose_received_ = absolute_odom_pose;
  last_transformed_odom_received_ = transformed_odom_pose;

  main_graph->addNewKeyframe(transformed_odom_pose, relative_odom_pose, _odometry_covariance);

  if (temp_graph_generated) {
    for (auto object : temp_graph->getObjectNodes()) {
      Eigen::Isometry3d absolute_obj_pose = object.second->getPose();
      Eigen::Isometry3d relative_obj_pose = absolute_odom_pose.inverse() * absolute_obj_pose;
      // FIXME: get object edge covariance
      // main_graph->addNewObjectKeyframe(object.first, absolute_obj_pose, relative_obj_pose,
      //                                  object.second->getCovariance());
      main_graph->addNewObjectKeyframe(
        object.first, absolute_obj_pose, relative_obj_pose,
        main_graph_object_covariance);
    }

    // FLAG("RESET TEMP GRAPH");
    auto sharing = temp_graph.use_count();
    if (sharing > 1) {
      DEBUG("Temp graph Shared: " << sharing);
    }
    temp_graph.reset();
    temp_graph = std::make_shared<GraphG2O>("Temp Graph");
    temp_graph_generated = false;
  }

  // TODO(dps): Choose when to optimize: either every time a new keyframe is added, or every certain
  // period of time
  main_graph->optimizeGraph();
  updateOdomMapTransform();
  // debugGraphVertices(main_graph);

  return true;
}

void OptimizerG2O::handleNewObject(
  const std::string & _obj_id,
  const Eigen::Isometry3d & _obj_pose,
  const Eigen::MatrixXd & _obj_covariance,
  const Eigen::Isometry3d & _new_odometry_pose,
  const Eigen::MatrixXd & _odometry_covariance)
{
  Eigen::Isometry3d absolute_odom_pose;
  Eigen::Isometry3d relative_odom_pose;
  generateRelativeAndAbsoluteOdometryPoses(
    _new_odometry_pose, absolute_odom_pose,
    relative_odom_pose);
  Eigen::Isometry3d transformed_odom_pose = map_odom_tranform_ * absolute_odom_pose;

  if (!temp_graph_generated) {
    temp_graph->initGraph(transformed_odom_pose);
    main_graph_object_covariance = _obj_covariance;
    temp_graph_generated = true;
  }

  // check distance from the last odometry received
  double translation_distance_from_last_node = relative_odom_pose.translation().norm();
  if (translation_distance_from_last_node < obj_odometry_distance_threshold_) {
    // std::cout << "New odometry distance is not enough" << std::endl;
    return;
  }
  // DEBUG("**** Adding new ODOM keyframe from Object detection ****");
  temp_graph->addNewKeyframe(transformed_odom_pose, relative_odom_pose, _odometry_covariance);

  // DEBUG("**** Added new OBJECT keyframe ****");
  Eigen::Isometry3d absolute_obj_pose;
  Eigen::Isometry3d relative_obj_pose;
  bool detections_are_absolute = false;

  if (detections_are_absolute) {
    absolute_obj_pose = _obj_pose;
    relative_obj_pose = absolute_odom_pose.inverse() * absolute_obj_pose;

  } else {
    relative_obj_pose = _obj_pose;
    absolute_obj_pose = transformed_odom_pose * relative_obj_pose;
  }

  // WARN("new_obj_pose");
  // auto pose = convertToPoseSE3(relative_obj_pose);
  // INFO(pose.position.x() << " " << pose.position.y() << " " << pose.position.z());
  // WARN("abs_obj_pose");
  // pose = convertToPoseSE3(absolute_obj_pose);
  // INFO(pose.position.x() << " " << pose.position.y() << " " << pose.position.z());

  temp_graph->addNewObjectKeyframe(_obj_id, absolute_obj_pose, relative_obj_pose, _obj_covariance);
  // temp_graph->optimizeGraph();

  // debugGraphVertices(temp_graph);
}

void OptimizerG2O::updateOdomMapTransform()
{
  map_odom_tranform_ = getOptimizedPose() * last_abs_odom_pose_received_.inverse();
}

bool OptimizerG2O::getNodePose(
  g2o::HyperGraph::Vertex * _node,
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> & _node_pose)
{
  g2o::VertexSE3 * node_se3 = dynamic_cast<g2o::VertexSE3 *>(_node);
  if (!node_se3) {
    return false;
  }

  _node_pose.first = node_se3->estimate().translation();
  _node_pose.second = Eigen::Quaterniond(node_se3->estimate().rotation());

  return true;
}

Eigen::Isometry3d OptimizerG2O::getOptimizedPose()
{
  return main_graph->getLastOdomNode()->getPose();
}

Eigen::Isometry3d OptimizerG2O::getMapOdomTransform()
{
  return map_odom_tranform_;
}
