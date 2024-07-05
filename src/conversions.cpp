/********************************************************************************************
 *  \file       conversions.cpp
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

#include "utils/conversions.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>

PoseSE3 convertToPoseSE3(const Eigen::Vector3d& _position, const Eigen::Quaterniond& _orientation) {
  PoseSE3 pose;
  pose.position    = _position;
  pose.orientation = _orientation;
  return pose;
}

PoseSE3 convertToPoseSE3(const geometry_msgs::msg::Pose& _pose) {
  PoseSE3 pose;
  pose.position    = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
  pose.orientation = Eigen::Quaterniond(_pose.orientation.w, _pose.orientation.x,
                                        _pose.orientation.y, _pose.orientation.z);
  return pose;
}

PoseSE3 convertToPoseSE3(Eigen::Isometry3d _isometry) {
  PoseSE3 pose;
  pose.position    = _isometry.translation();
  pose.orientation = Eigen::Quaterniond(_isometry.rotation());
  return pose;
}

Eigen::Isometry3d getIsometry(Eigen::Vector3d _position, Eigen::Quaterniond _orientation) {
  Eigen::Isometry3d isometry = Eigen::Translation3d(_position) * _orientation;
  return isometry;
}

geometry_msgs::msg::Pose convertToGeometryMsgPose(const Eigen::Isometry3d& _isometry) {
  geometry_msgs::msg::Pose geometry_msg_pose;
  PoseSE3 pose                    = convertToPoseSE3(_isometry);
  geometry_msg_pose.position.x    = pose.position.x();
  geometry_msg_pose.position.y    = pose.position.y();
  geometry_msg_pose.position.z    = pose.position.z();
  geometry_msg_pose.orientation.w = pose.orientation.w();
  geometry_msg_pose.orientation.x = pose.orientation.x();
  geometry_msg_pose.orientation.y = pose.orientation.y();
  geometry_msg_pose.orientation.z = pose.orientation.z();
  return geometry_msg_pose;
}
