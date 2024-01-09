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
 *  MeshGoalTool.cpp
 *
 *  author: Alexander Mock <amock@uos.de>
 */


#include "rviz_mesh_tools_plugins/MeshPoseGuessTool.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz_mesh_tools_plugins::MeshPoseGuessTool, rviz_common::Tool )

namespace rviz_mesh_tools_plugins
{

MeshPoseGuessTool::MeshPoseGuessTool()
:qos_profile_(5)
{
  shortcut_key_ = 'e';
  topic_property_ = new rviz_common::properties::StringProperty( "Topic", "initialpose",
                                              "The topic on which to publish the mesh pose estimate.",
                                              getPropertyContainer(), SLOT(updateTopic()), this);
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);

  switch_bottom_top_ = new rviz_common::properties::BoolProperty("Switch Bottom/Top",
      false, "Enable to stwich the bottom and top.",
      getPropertyContainer());


  cov_position_x_property_ = new rviz_common::properties::FloatProperty("Covariance position x", 0.5f * 0.5f, "Covariance ppositionos x", getPropertyContainer());
  cov_position_y_property_ = new rviz_common::properties::FloatProperty("Covariance position y", 0.5f * 0.5f, "Covariance position y", getPropertyContainer());
  cov_position_z_property_ = new rviz_common::properties::FloatProperty("Covariance position z", 0.1f * 0.1f, "Covariance position z", getPropertyContainer());
  cov_orientation_x_property_ = new rviz_common::properties::FloatProperty("Covariance orientation x", 0.1f * 0.1f, "Covariance orientation x", getPropertyContainer());
  cov_orientation_y_property_ = new rviz_common::properties::FloatProperty("Covariance orientation y", 0.1f * 0.1f, "Covariance orientation y", getPropertyContainer());
  cov_orientation_z_property_ = new rviz_common::properties::FloatProperty("Covariance orientation z", static_cast<float>(M_PI / 12.0 * M_PI / 12.0), "Covariance orientation z", getPropertyContainer());
}

void MeshPoseGuessTool::onInitialize()
{
  MeshPoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("Mesh Pose Estimate");
  updateTopic();

  setColor(1.0f, 0.0f, 0.0f, 1.0f);
}

void MeshPoseGuessTool::updateTopic()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_property_->getStdString(), qos_profile_);
  clock_ = node->get_clock();
}
  
void MeshPoseGuessTool::onPoseSet( 
  const Ogre::Vector3& position, 
  const Ogre::Quaternion& orientation )
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.pose.pose.position.x = position.x;
  msg.pose.pose.position.y = position.y;
  msg.pose.pose.position.z = position.z;
  
  // ogreToRos(x,y,z) = (-z,-x,y) 

  Ogre::Quaternion ros_orientation;

  if(switch_bottom_top_->getBool())
  {
    ros_orientation.FromAxes(
        -orientation.zAxis(),
        orientation.xAxis(),
        -orientation.yAxis()
    );
  }
  else
  {
    ros_orientation.FromAxes(
        -orientation.zAxis(),
        -orientation.xAxis(),
        orientation.yAxis()
    );
  }

  msg.pose.pose.orientation.x = ros_orientation.x;
  msg.pose.pose.orientation.y = ros_orientation.y;
  msg.pose.pose.orientation.z = ros_orientation.z;
  msg.pose.pose.orientation.w = ros_orientation.w;

  for(size_t i=0; i<36; i++)
  {
    msg.pose.covariance[i] = 0.0;
  }

  msg.pose.covariance[0*6+0] = cov_position_x_property_->getFloat();
  msg.pose.covariance[1*6+1] = cov_position_y_property_->getFloat();
  msg.pose.covariance[2*6+2] = cov_position_z_property_->getFloat();
  msg.pose.covariance[3*6+3] = cov_orientation_x_property_->getFloat();
  msg.pose.covariance[4*6+4] = cov_orientation_y_property_->getFloat();
  msg.pose.covariance[5*6+5] = cov_orientation_z_property_->getFloat();
  
  msg.header.stamp = clock_->now();
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  pose_pub_->publish(msg);
}
	
} // namespace rviz_mesh_tools_plugins
