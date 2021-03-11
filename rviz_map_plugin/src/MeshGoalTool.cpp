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
 *  MeshGoalTool.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */


#include "MeshGoalTool.hpp"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_map_plugin::MeshGoalTool, rviz::Tool )

namespace rviz_map_plugin{
MeshGoalTool::MeshGoalTool()
{
  shortcut_key_ = 'm';
  topic_property_ = new rviz::StringProperty( "Topic", "goal",
                                              "The topic on which to publish the mesh navigation goals.",
                                              getPropertyContainer(), SLOT(updateTopic()), this);

  switch_bottom_top_ = new rviz::BoolProperty("Switch Bottom/Top",
      false, "Enable to stwich the bottom and top.",
      getPropertyContainer());

}
 
 void MeshGoalTool::onInitialize()
 {
   MeshPoseTool::onInitialize();
   setName( "Mesh Goal" );
   updateTopic();
 }

 void MeshGoalTool::updateTopic()
 {
   pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );
 }
  
 void MeshGoalTool::onPoseSet( const Ogre::Vector3& position, const Ogre::Quaternion& orientation ){
  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = position.x;
  msg.pose.position.y = position.y;
  msg.pose.position.z = position.z;
  
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

  msg.pose.orientation.x = ros_orientation.x;
  msg.pose.orientation.y = ros_orientation.y;
  msg.pose.orientation.z = ros_orientation.z;
  msg.pose.orientation.w = ros_orientation.w;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  pose_pub_.publish(msg);
 }
	
}
