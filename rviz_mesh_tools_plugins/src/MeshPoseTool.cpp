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
 *  MeshPoseTool.cpp
 *
 *	authors:  Sebastian Pütz <spuetz@uos.de>
 *            Alexander Mock <amock@uos.de>
 *
 */

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>

#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include "rviz_rendering/render_window.hpp"

// #include "rviz_rendering/render_window.hpp"

#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_controller.hpp>

// method is private :(
// #include <rviz_rendering/viewport_projection_finder.hpp>
#include <rviz_common/interaction/selection_manager_iface.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>

#include <cassert>

#include "rviz_mesh_tools_plugins/MeshPoseTool.hpp"

#include <rviz_mesh_tools_plugins/InteractionHelper.hpp>


namespace rviz_mesh_tools_plugins
{

MeshPoseTool::MeshPoseTool() 
:rviz_common::Tool()
,arrow_(NULL)
{
}

MeshPoseTool::~MeshPoseTool()
{
  if(arrow_)
  {
    delete arrow_;
  }
}

void MeshPoseTool::onInitialize()
{
  arrow_ = new rviz_rendering::Arrow(scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void MeshPoseTool::activate()
{
  setStatus("Click and on a mesh_msgs::TriangleMesh to set the position and drag the mouse for the orientation.");
  state_ = Position;
}

void MeshPoseTool::setColor(float r, float g, float b, float a)
{
  arrow_->setColor(r, g, b, a);
}

void MeshPoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int MeshPoseTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  int flags = 0;

  if (event.leftDown())
  {
    // TODO: check if there is a proper ROS2 equivalent
    // RCLCPP_ASSERT(state_ == Position);
    assert(state_ == Position);

    Ogre::Ray mouse_ray = getMouseEventRay(event);
    
    // Find intersection
    Ogre::Vector3 int_pos;
    Ogre::Vector3 int_normal;
    Intersection intersection;
    if(selectFace(context_, mouse_ray, intersection))
    {
      pos_start_ = intersection.point;
      normal_start_ = intersection.normal;

      // Flip normal to camera
      Ogre::Vector3 cam_pos = event.panel->getViewController()->getCamera()->getRealPosition();
      Ogre::Vector3 dir_to_cam = cam_pos - pos_start_;
      dir_to_cam.normalise();
      // there should be a function doing the dot product
      float scalar = dir_to_cam.x * normal_start_.x 
                  + dir_to_cam.y * normal_start_.y 
                  + dir_to_cam.z * normal_start_.z;

      if(scalar < 0)
      {
        normal_start_ = -normal_start_;
      }

      arrow_->setPosition(pos_start_);
      state_ = Orientation;
    }
  }
  else if (event.type == QEvent::MouseMove && event.left())
  {
    if (state_ == Orientation)
    {
      Ogre::Plane plane(normal_start_, pos_start_);
      Ogre::Ray mouse_ray = getMouseEventRay(event);

      // intersect with plane
      auto res = mouse_ray.intersects(plane);
      if(res.first)
      {
        // plane hit
        pos_last_ = mouse_ray.getPoint(res.second);
        if( (pos_last_ - pos_start_).squaredLength() > 0.001 * 0.001 )
        {
          // valid distance

          // right hand coordinate system
          // x to the right, y to the top, and -z into the scene
          // arrow foreward negative z
          Ogre::Vector3 z_axis = -(pos_last_ - pos_start_).normalisedCopy();
          Ogre::Vector3 y_axis = normal_start_;
          Ogre::Vector3 x_axis = y_axis.crossProduct(z_axis).normalisedCopy();
          
          arrow_->getSceneNode()->setVisible(true);
          arrow_->setOrientation(Ogre::Quaternion(x_axis, y_axis, z_axis));

          flags |= Render;
        } 
      }

    }
  }
  else if (event.leftUp())
  {
    if (state_ == Orientation)
    {
      Ogre::Plane plane(normal_start_, pos_start_);
      Ogre::Ray mouse_ray = getMouseEventRay(event);

      // intersect with plane
      auto res = mouse_ray.intersects(plane);
      if(res.first)
      {
        // plane hit
        pos_last_ = mouse_ray.getPoint(res.second);
        if( (pos_last_ - pos_start_).squaredLength() > 0.001 * 0.001 )
        {
          // valid distance

          // right hand coordinate system
          // x to the right, y to the top, and -z into the scene
          // arrow foreward negative z
          Ogre::Vector3 z_axis = -(pos_last_ - pos_start_).normalisedCopy();
          Ogre::Vector3 y_axis = normal_start_;
          Ogre::Vector3 x_axis = y_axis.crossProduct(z_axis).normalisedCopy();
          
          arrow_->getSceneNode()->setVisible(true);
          arrow_->setOrientation(Ogre::Quaternion(x_axis, y_axis, z_axis));

          onPoseSet(pos_start_, Ogre::Quaternion(x_axis, y_axis, z_axis));

          flags |= (Finished | Render);
          state_ = Position;
        }
      }
    }
  }

  return flags;
}

}  // namespace rviz_mesh_tools_plugins
