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
 *  MeshPoseTool.hpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef MESH_POSE_TOOL_HPP
#define MESH_POSE_TOOL_HPP

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRay.h>

#include <QCursor>
#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/ogre_helpers/arrow.h>

namespace rviz_map_plugin
{
class MeshPoseTool : public rviz::Tool
{
public:
  MeshPoseTool();
  virtual ~MeshPoseTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

protected:
  virtual void onPoseSet(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) = 0;

  void getRawManualObjectData(const Ogre::ManualObject* mesh, const size_t sectionNumber, size_t& vertexCount,
                              Ogre::Vector3*& vertices, size_t& indexCount, unsigned long*& indices);

  bool getPositionAndOrientation(const Ogre::ManualObject* mesh, const Ogre::Ray& ray, Ogre::Vector3& position,
                                 Ogre::Vector3& orientation);

  bool selectTriangle(rviz::ViewportMouseEvent& event, Ogre::Vector3& position, Ogre::Vector3& orientation);

  rviz::Arrow* arrow_;
  enum State
  {
    Position,
    Orientation
  };
  State state_;
  Ogre::Vector3 pos_;
  Ogre::Vector3 ori_;
};

} /* namespace rviz_map_plugin */

#endif
