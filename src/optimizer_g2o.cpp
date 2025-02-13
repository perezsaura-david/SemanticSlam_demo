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
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "as2_slam/graph_g2o.hpp"
#include "as2_slam/graph_node_types.hpp"
#include "as2_slam/object_detection_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"
#include "utils/debug_utils.hpp"

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
  last_odom_pose_added_ = Eigen::Isometry3d::Identity();
}

bool OptimizerG2O::generateOdometryInfo(
  const Eigen::Isometry3d & _new_odometry_pose, const Eigen::Isometry3d _last_odom_pose_added,
  OdometryInfo & _odometry_info)
{
  if (odometry_is_relative_) {
    // TODO(dps): RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return false;
  } else {
    // ABSOLUTE ODOMETRY
    _odometry_info.odom_ref = _new_odometry_pose;
    _odometry_info.increment = _last_odom_pose_added.inverse() * _odometry_info.odom_ref;
  }
  _odometry_info.map_ref = map_odom_tranform_ * _odometry_info.odom_ref;
  return true;
}

bool OptimizerG2O::handleNewOdom(
  const Eigen::Isometry3d & _new_odometry_pose,
  const Eigen::MatrixXd & _odometry_covariance)
{
  OdometryInfo new_odometry_info;
  generateOdometryInfo(_new_odometry_pose, last_odom_pose_added_, new_odometry_info);

  if (!checkAddingConditions(new_odometry_info, odometry_distance_threshold_)) {
    return false;
  }
  // INFO("New odometry distance is enough: " << translation_distance_from_last_node);
  last_odom_pose_added_ = new_odometry_info.odom_ref;

  main_graph->addNewKeyframe(
    new_odometry_info.map_ref, new_odometry_info.increment,
    _odometry_covariance);

  if (temp_graph_generated_) {
    temp_graph->optimizeGraph();
    // FLAG("ADD TEMP GRAPH DETECTIONS TO MAIN GRAPH");
    for (auto object : temp_graph->getObjectNodes()) {
      // TODO(dps): Get all the nodes related and create new edges
      ObjectDetection * object_detection;
      ArucoNode * aruco_node = dynamic_cast<ArucoNode *>(object.second);
      if (aruco_node) {
        // Eigen::MatrixXd cov_matrix = aruco_node->getOptimizedInformationMatrix().inverse();
        Eigen::MatrixXd cov_matrix = main_graph_object_covariance;

        object_detection = new ArucoDetection(
          object.first,
          aruco_node->getPose(), cov_matrix, true);
      }
      GateNode * gate_node = dynamic_cast<GateNode *>(object.second);
      if (gate_node) {
        // Eigen::MatrixXd cov_matrix = main_graph_object_covariance;
        // Eigen::MatrixXd cov_matrix = gate_node->getOptimizedInformationMatrix().inverse();
        Eigen::MatrixXd cov_matrix = temp_graph->computeNodeCovariance(gate_node);
        if (cov_matrix.size() == 0) {
          WARN("Matrix is empty! Using default matrix");
          cov_matrix = main_graph_object_covariance;
        }
        // INFO(PRINT_VAR(cov_matrix));

        object_detection = new GateDetection(
          object.first,
          gate_node->getPosition(), cov_matrix, true);
      }

      if (!object_detection->prepareMeasurements(new_odometry_info)) {
        ERROR("Prepare detection ERROR");
        continue;
      }
      // FIXME(dps): get object edge covariance
      main_graph->addNewObjectDetection(object_detection);
    }

    // FLAG("RESET TEMP GRAPH");
    auto sharing = temp_graph.use_count();
    if (sharing > 1) {
      DEBUG("Temp graph Shared: " << sharing);
    }
    temp_graph.reset();
    temp_graph = std::make_shared<GraphG2O>("Temp Graph");
    temp_graph_generated_ = false;
  }

  // TODO(dps): Choose when to optimize: either every time a new keyframe is added, or every certain
  // period of time
  main_graph->optimizeGraph();
  if (generate_odom_map_transform_) {
    updateOdomMapTransform();
  }
  // debugGraphVertices(main_graph);

  return true;
}

bool OptimizerG2O::checkAddingConditions(
  const OdometryInfo & _odometry, const double _distance_threshold)
{
  // TODO(dps): check time from the last odometry received
  // FIXME(dps): get rotation distance
  double translation_distance_from_last_node = _odometry.increment.translation().norm();
  if (translation_distance_from_last_node < _distance_threshold) {
    // std::cout << "New odometry distance is not enough" << std::endl;
    return false;
  }
  return true;
}

// TODO(dps): return bool?
void OptimizerG2O::handleNewObjectDetection(
  ObjectDetection * _object,
  const Eigen::Isometry3d & _odometry_pose,
  const Eigen::MatrixXd & _odometry_covariance)
{
  if (!temp_graph_generated_) {
    last_obj_odom_pose_added_ = last_odom_pose_added_;
  }

  OdometryInfo detection_odometry;
  generateOdometryInfo(_odometry_pose, last_obj_odom_pose_added_, detection_odometry);

  if (!temp_graph_generated_) {
    temp_graph->initGraph(detection_odometry.map_ref);
    main_graph_object_covariance = _object->getCovarianceMatrix();  // FIXME(dps): remove this
    temp_graph_generated_ = true;
    // last_obj_odom_pose_added_ = detection_odometry.odom_ref;
  } else {
    if (!checkAddingConditions(detection_odometry, obj_odometry_distance_threshold_)) {
      // std::cout << "New odometry distance is not enough" << std::endl;
      return;
    }
    // last_obj_odom_pose_added_ = detection_odometry.odom_ref;
    // DEBUG("**** Adding new ODOM keyframe from Object detection ****");
    temp_graph->addNewKeyframe(
      detection_odometry.map_ref, detection_odometry.increment,
      _odometry_covariance);
  }

  last_obj_odom_pose_added_ = detection_odometry.odom_ref;

  if (!_object->prepareMeasurements(detection_odometry)) {
    ERROR("Prepare detection ERROR");
    return;
  }

  temp_graph->addNewObjectDetection(_object);
  // debugGraphVertices(temp_graph);
  // temp_graph->optimizeGraph();
  // debugGraphVertices(temp_graph);
}

void OptimizerG2O::updateOdomMapTransform()
{
  map_odom_tranform_ = getOptimizedPose() * last_odom_pose_added_.inverse();
}

Eigen::Isometry3d OptimizerG2O::getOptimizedPose()
{
  return main_graph->getLastOdomNode()->getPose();
}

Eigen::Isometry3d OptimizerG2O::getMapOdomTransform()
{
  return map_odom_tranform_;
}
