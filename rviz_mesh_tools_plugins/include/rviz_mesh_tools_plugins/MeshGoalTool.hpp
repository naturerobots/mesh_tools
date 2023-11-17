/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
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
 *  MeshGoalTool.hpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef MESH_GOAL_TOOL_HPP
#define MESH_GOAL_TOOL_HPP

#include <rviz_mesh_tools_plugins/MeshPoseTool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/display_context.hpp>

#ifndef Q_MOC_RUN
#include <QObject>
#endif // Q_MOC_RUN

#include <rclcpp/rclcpp.hpp>

namespace rviz_mesh_tools_plugins
{
/**
 * @class MeshGoalTool
 * @brief Tool for publishing a goal within a mesh
 */
class MeshGoalTool : public MeshPoseTool
{
  Q_OBJECT
public:
  /**
   * @brief Constructor
   */
  MeshGoalTool();

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

  /// Property for the topic
  rviz_common::properties::StringProperty* topic_property_;
  /// Switch bottom / top for selection
  rviz_common::properties::BoolProperty* switch_bottom_top_;
  /// Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  /// Node handle
  // ros::NodeHandle nh_;
};

} // namespace rviz_mesh_tools_plugins

#endif // MESH_GOAL_TOOL_HPP
