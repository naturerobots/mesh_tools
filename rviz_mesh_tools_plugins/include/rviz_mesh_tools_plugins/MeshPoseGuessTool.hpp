/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2023, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  MeshPoseGuessTool.hpp
 *
 *  author: Alexander Mock
 */

#ifndef MESH_POSE_ESTIMATE_TOOL_HPP
#define MESH_POSE_ESTIMATE_TOOL_HPP

#include <rviz_mesh_tools_plugins/MeshPoseTool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rviz_common/display_context.hpp>


#ifndef Q_MOC_RUN
#include <QObject>
#endif // Q_MOC_RUN

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"

namespace rviz_common
{

namespace properties
{

class StringProperty;
class QosProfileProperty;
class BoolProperty;
class FloatProperty;

} // namespace properties

} // namespace rviz_common

namespace rviz_mesh_tools_plugins
{
/**
 * @class MeshPoseGuessTool
 * @brief Tool for publishing a pose estimate within a mesh
 */
class MeshPoseGuessTool : public MeshPoseTool
{
  Q_OBJECT
public:
  /**
   * @brief Constructor
   */
  MeshPoseGuessTool();

  /**
   * @brief Callback that is executed when tool is initialized
   */
  virtual void onInitialize();

private Q_SLOTS:

  /**
   * @brief Updates the topic on which the goal will be published
   */
  void updateTopic();

protected:
  /**
   * @brief When goal is set, publish result
   * @param position Position
   * @param orientation Orientation
   */
  virtual void onPoseSet(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

  /// Property for the topic name
  rviz_common::properties::StringProperty* topic_property_;

  // Properties for QosProfile
  rviz_common::properties::QosProfileProperty* qos_profile_property_;

  /// Switch bottom / top for selection
  rviz_common::properties::BoolProperty* switch_bottom_top_;

  // Covariance?
  rviz_common::properties::FloatProperty*  cov_position_x_property_;
  rviz_common::properties::FloatProperty*  cov_position_y_property_;
  rviz_common::properties::FloatProperty*  cov_position_z_property_;
  rviz_common::properties::FloatProperty*  cov_orientation_x_property_;
  rviz_common::properties::FloatProperty*  cov_orientation_y_property_;
  rviz_common::properties::FloatProperty*  cov_orientation_z_property_;

  /// Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::QoS qos_profile_;
  rclcpp::Clock::SharedPtr clock_;
};

} // namespace rviz_mesh_tools_plugins

#endif // MESH_POSE_ESTIMATE_TOOL_HPP
