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



namespace rviz_mesh_tools_plugins
{

// bool getViewportProjectionOnPlane(
//   rviz_rendering::RenderWindow* render_window,
//   int x, int y,
//   const Ogre::Plane& plane,
//   Ogre::Vector3& intersection_point)
// {
//   auto viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window);
//   int width = viewport->getActualWidth();
//   int height = viewport->getActualHeight();
//   Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
//     static_cast<float>(x) / static_cast<float>(width),
//     static_cast<float>(y) / static_cast<float>(height));

//   auto intersection = mouse_ray.intersects(plane);
//   if (!intersection.first) 
//   {
//     return false;
//   }

//   intersection_point = mouse_ray.getPoint(intersection.second);
//   return true;
// }

MeshPoseTool::MeshPoseTool() : rviz_common::Tool(), arrow_(NULL)
{
}

MeshPoseTool::~MeshPoseTool()
{
  delete arrow_;
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

    auto picker = context_->getViewPicker();

    Ogre::Vector3 cur_pos;
    if(picker->get3DPoint(event.panel, event.x, event.y, cur_pos))
    {
      pos_start_ = cur_pos;


      std::vector<Ogre::Vector3> results;
      unsigned patch_size = smooth_normal_width_;
      bool success = picker->get3DPatch(event.panel, event.x, event.y, patch_size, patch_size, true, results);

      unsigned patch_middle = (patch_size * patch_size) / 2;
      if(success && results.size() > 7)
      {
        // std::cout << "Found " << results.size() << " results" << std::endl;

        // PCA
        // compute centroid
        Ogre::Vector3 centroid(0.0, 0.0, 0.0);
        for(size_t i=0; i<results.size(); i++)
        {
          centroid += results[i];
        }
        centroid /= static_cast<float>(results.size());

        Ogre::Matrix3 cov;
        cov[0][0] = 0.0; cov[0][1] = 0.0; cov[0][2] = 0.0;
        cov[1][0] = 0.0; cov[1][1] = 0.0; cov[1][2] = 0.0;
        cov[2][0] = 0.0; cov[2][1] = 0.0; cov[2][2] = 0.0;

        for(size_t i=0; i<results.size(); i++)
        {
          Ogre::Vector3 dir = results[i] - centroid;
          cov[0][0] += dir.x * dir.x;
          cov[0][1] += dir.x * dir.y;
          cov[0][2] += dir.x * dir.z;
          cov[1][0] += dir.y * dir.x;
          cov[1][1] += dir.y * dir.y;
          cov[1][2] += dir.y * dir.z;
          cov[2][0] += dir.z * dir.x;
          cov[2][1] += dir.z * dir.y;
          cov[2][2] += dir.z * dir.z;
        }

        cov = cov * Ogre::Real(1.0 / static_cast<float>(results.size()));

        float eig_vals[3];
        Ogre::Vector3 eig_vecs[3];
        cov.EigenSolveSymmetric(eig_vals, eig_vecs);

        // std::cout << "Eigen Analysis: " << std::endl;


        // for(size_t i=0; i<3; i++)
        // {
        //   std::cout << i << ". " << eig_vals[i] << std::endl;
        //   std::cout << eig_vecs[i] << std::endl;
        // }

        normal_start_ = eig_vecs[2];
        pos_start_ = results[patch_middle];

        normal_start_.normalise();

        // flip normal to cam

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

  }
  else if (event.type == QEvent::MouseMove && event.left())
  {
    if (state_ == Orientation)
    {
      auto picker = context_->getViewPicker();

      Ogre::Vector3 cur_pos;
      if(picker->get3DPoint(event.panel, event.x, event.y, cur_pos))
      {
        // if valid override last pos
        if( (cur_pos - pos_start_).squaredLength() > 0.001 * 0.001)
        {
          pos_last_ = cur_pos;
        }

        // if last pos is valid compute orientation
        if( (pos_last_ - pos_start_).squaredLength() > 0.001 * 0.001 )
        {
          // compute orientation
          Ogre::Vector3 dir_front = pos_last_ - pos_start_;
          dir_front.normalise();
          // std::cout << "Dir Front: " << dir_front << std::endl;

          Ogre::Vector3 dir_left = normal_start_.crossProduct(dir_front);
          dir_left.normalise();
          // std::cout << "Dir Left: " << dir_left << std::endl;

          Ogre::Vector3 dir_up = dir_front.crossProduct(dir_left);
          dir_up.normalise();
          // std::cout << "Dir Up: " << dir_up << std::endl; 


          arrow_->getSceneNode()->setVisible(true);
          arrow_->setOrientation(Ogre::Quaternion(dir_up, dir_left, -dir_front));

          flags |= Render;
        }
      }
    }
  }
  else if (event.leftUp())
  {
    if (state_ == Orientation)
    {

      auto picker = context_->getViewPicker();

      Ogre::Vector3 cur_pos;
      if(picker->get3DPoint(event.panel, event.x, event.y, cur_pos))
      {
        // if valid override last pos
        if( (cur_pos - pos_start_).squaredLength() > 0.001 * 0.001)
        {
          pos_last_ = cur_pos;
        }

        // if last pos is valid compute orientation
        if( (pos_last_ - pos_start_).squaredLength() > 0.001 * 0.001 )
        {
          // compute orientation
          Ogre::Vector3 dir_front = pos_last_ - pos_start_;
          dir_front.normalise();
          // std::cout << "Dir Front: " << dir_front << std::endl;
          

          Ogre::Vector3 dir_left = dir_front.crossProduct(normal_start_);
          dir_left.normalise();
          // std::cout << "Dir Left: " << dir_left << std::endl;

          Ogre::Vector3 dir_up = dir_front.crossProduct(dir_left);
          dir_up.normalise();
          // std::cout << "Dir Up: " << dir_up << std::endl; 

          arrow_->getSceneNode()->setVisible(true);
          arrow_->setOrientation(Ogre::Quaternion(dir_up, dir_left, -dir_front));
          
          onPoseSet(pos_start_, Ogre::Quaternion(dir_up, dir_left, -dir_front));

          flags |= (Finished | Render);
          state_ = Position;
        }
      }
    }
  }

  return flags;
}

bool MeshPoseTool::selectTriangle(
  rviz_common::ViewportMouseEvent& event, 
  Ogre::Vector3& position,
  Ogre::Vector3& triangle_normal)
{
  // Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay(
  //     (float)event.x / event.viewport->getActualWidth(), (float)event.y / event.viewport->getActualHeight());

  // rviz_common::interaction::Picked pick_result;

  // auto manager = context_->getSelectionManager();

  // bool hit = manager->pick(
  //   event.panel->getRenderWindow(),
  //   event.x, event.y,
  //   pick_result);
  
  // if(hit)
  // {
  //   std::cout << "HIT" << std::endl;
  // } else {
  //   std::cout << "NO HIT" << std::endl;
  // }


  auto picker = context_->getViewPicker();

  Ogre::Vector3 int_point;
  if(picker->get3DPoint(event.panel, event.x, event.y, int_point))
  {
    std::cout << "HIT!" << std::endl;
  } else {
    std::cout << "NOT HIT :(" << std::endl;
  }

  


  Ogre::Ray ray;

  Ogre::RaySceneQuery* query =
      context_->getSceneManager()->createRayQuery(ray, Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
  
  
  query->setSortByDistance(true);

  Ogre::RaySceneQueryResult& result = query->execute();

  for (size_t i = 0; i < result.size(); i++)
  {
    if (result[i].movable->getName().find("TriangleMesh") != std::string::npos)
    {
      Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(result[i].movable);
      size_t goal_section = -1;
      size_t goal_index = -1;
      Ogre::Real dist = -1;
      if (getPositionAndOrientation(mesh, ray, position, triangle_normal))
      {
        return true;
      }
    }
  }
  return false;
}

bool MeshPoseTool::getPositionAndOrientation(
  const Ogre::ManualObject* mesh, 
  const Ogre::Ray& ray,
  Ogre::Vector3& position, Ogre::Vector3& orientation)
{
  Ogre::Real dist = -1.0f;
  Ogre::Vector3 a, b, c;

  size_t vertex_count = 0;
  Ogre::Vector3* vertices;
  size_t index_count = 0;
  unsigned long* indices;
  size_t num_sections = mesh->getNumSections();

  for (size_t i = 0; i < num_sections; i++)
  {
    getRawManualObjectData(mesh, i, vertex_count, vertices, index_count, indices);
    if (index_count != 0)
    {
      for (size_t j = 0; j < index_count; j += 3)
      {
        std::pair<bool, Ogre::Real> goal = Ogre::Math::intersects(ray, vertices[indices[j]], vertices[indices[j + 1]],
                                                                  vertices[indices[j + 2]], true, true);

        if (goal.first)
        {
          if ((dist < 0.0f) || (goal.second < dist))
          {
            dist = goal.second;
            a = vertices[indices[j]];
            b = vertices[indices[j + 1]];
            c = vertices[indices[j + 2]];
          }
        }
      }
    }
  }

  delete[] vertices;
  delete[] indices;
  if (dist != -1)
  {
    position = ray.getPoint(dist);
    Ogre::Vector3 ab = b - a;
    Ogre::Vector3 ac = c - a;
    orientation = ac.crossProduct(ab).normalisedCopy();
    return true;
  }
  else
  {
    return false;
  }
}

void MeshPoseTool::getRawManualObjectData(const Ogre::ManualObject* mesh, const size_t sectionNumber,
                                          size_t& vertexCount, Ogre::Vector3*& vertices, size_t& indexCount,
                                          unsigned long*& indices)
{
  Ogre::VertexData* vertexData;
  const Ogre::VertexElement* vertexElement;
  Ogre::HardwareVertexBufferSharedPtr vertexBuffer;
  unsigned char* vertexChar;
  float* vertexFloat;

  vertexData = mesh->getSection(sectionNumber)->getRenderOperation()->vertexData;
  vertexElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
  vertexBuffer = vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());
  vertexChar = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

  vertexCount = vertexData->vertexCount;
  vertices = new Ogre::Vector3[vertexCount];

  for (size_t i = 0; i < vertexCount; i++, vertexChar += vertexBuffer->getVertexSize())
  {
    vertexElement->baseVertexPointerToElement(vertexChar, &vertexFloat);
    vertices[i] =
        (mesh->getParentNode()->_getDerivedOrientation() *
         (Ogre::Vector3(vertexFloat[0], vertexFloat[1], vertexFloat[2]) * mesh->getParentNode()->_getDerivedScale())) +
        mesh->getParentNode()->_getDerivedPosition();
  }

  vertexBuffer->unlock();

  Ogre::IndexData* indexData;
  Ogre::HardwareIndexBufferSharedPtr indexBuffer;
  indexData = mesh->getSection(sectionNumber)->getRenderOperation()->indexData;
  indexCount = indexData->indexCount;
  indices = new unsigned long[indexCount];
  indexBuffer = indexData->indexBuffer;
  unsigned int* pLong = static_cast<unsigned int*>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
  unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

  for (size_t i = 0; i < indexCount; i++)
  {
    unsigned long index;
    if (indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
    {
      index = static_cast<unsigned long>(pLong[i]);
    }
    else
    {
      index = static_cast<unsigned long>(pShort[i]);
    }

    indices[i] = index;
  }
  indexBuffer->unlock();
}

}  // namespace rviz_mesh_tools_plugins
