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
 *  ClusterLabelVisual.hpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 */

#ifndef CLUSTER_LABEL_VISUAL_HPP
#define CLUSTER_LABEL_VISUAL_HPP

#include <Types.hpp>

#include <ros/ros.h>

#include <rviz/display.h>

#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/MeshGeometry.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreMesh.h>
#include <OGRE/OgreSubMesh.h>

#include <memory>
#include <vector>

namespace Ogre
{
// Forward declaration
class SceneNode;
class Mesh;
}  // End namespace Ogre

namespace rviz_map_plugin
{
/**
 * @class ClusterLabelVisual
 * @brief Visual to show a labeled cluster
 */
class ClusterLabelVisual
{
public:
  /**
   * @brief Constructor
   *
   * @param context The context that contains the display information.
   * @param labelId The label id (that has to be unique)
   */
  ClusterLabelVisual(rviz::DisplayContext* context, std::string labelId);

  /**
   * @brief Constructor
   *
   * @param context The context that contains the display information.
   * @param labelId The label id (that has to be unique)
   * @param geometry A shared pointer to the geometry to which the labels belong
   */
  ClusterLabelVisual(rviz::DisplayContext* context, std::string labelId, std::shared_ptr<Geometry> geometry);

  /**
   * @brief Destructor
   */
  ~ClusterLabelVisual();

  /**
   * @brief Disabling the copy constructor
   *
   * Each cluster label visual has a pointer to a SubMesh with a unique name,
   * when copying and then deleting one of the copies, the SubMesh would be deleted, thus the
   * pointers of the remaining copies would be invalid
   */
  ClusterLabelVisual(const ClusterLabelVisual&) = delete;

  /**
   * @brief Disabling the copy assignment operator
   *
   * explanation: see deleted copy constructor ClusterLabelVisual(const ClusterLabelVisual&)
   */
  ClusterLabelVisual& operator=(const ClusterLabelVisual&) = delete;

  /**
   * @brief Deletes the material
   */
  void reset();

  /**
   * @brief Sets the geometry
   *
   * @param geometry The geometry
   */
  void setGeometry(std::shared_ptr<Geometry> geometry);

  /**
   * @brief Sets the faces, that are in the shown cluster
   *
   * @param faces A vector containing the face ids
   */
  void setFacesInCluster(const std::vector<uint32_t>& faces);

  /**
   * @brief Sets the color
   *
   * @param facesColor The color for the faces
   * @param alpha The opacity, defaults to 1.0f (fully opaque)
   */
  void setColor(Ogre::ColourValue facesColor, float alpha = 1.0f);

  /**
   * @brief Returns the faces
   *
   * @return A vector containing the face ids
   */
  std::vector<uint32_t> getFaces()
  {
    return m_faces;
  };

private:
  void initMaterial();

  rviz::DisplayContext* m_displayContext;
  Ogre::SceneNode* m_sceneNode;
  std::string m_labelId;

  Ogre::MeshPtr m_mesh;
  Ogre::SubMesh* m_subMesh;
  Ogre::MaterialPtr m_material;

  Ogre::ColourValue m_color;

  std::shared_ptr<Geometry> m_geometry;
  std::vector<uint32_t> m_faces;
};

}  // end namespace rviz_map_plugin

#endif