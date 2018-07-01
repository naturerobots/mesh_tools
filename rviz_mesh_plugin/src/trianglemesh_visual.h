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
 *  trianglemesh_visual.h
 *
 *
 *  authors:
 *
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 */

#ifndef TRIANGLEMESH_VISUAL_H
#define TRIANGLEMESH_VISUAL_H

#include <mesh_msgs/TriangleMeshStamped.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreColourValue.h>

namespace Ogre
{

// Forward declaration
class Vector3;
class Quaternion;
class SceneNode;
class Entity;

} // End namespace Ogre

namespace rviz_mesh_plugin
{

/**
 * @brief Class to display mesh data in the main panel of rviz.
 */
class TriangleMeshVisual
{
public:

    /**
     * @brief Constructor.
     *
     * @param context The context that contains the display information.
     * @param displayID The display id
     * @param meshID The mesh id
     * @param randomID random number that will be used as part of the meshes UID
     */
    TriangleMeshVisual(rviz::DisplayContext* context,
                       size_t displayID,
                       size_t meshID,
                       size_t randomID);

    /**
     * @brief Destructor.
     */
    virtual ~TriangleMeshVisual();

    /**
     * @brief Clears whole stored data.
     */
    void reset();

    /**
     * @brief Extracts data from the ros-messages and creates meshes.
     *
     * @param meshMsg         Message containing the mesh
     */
    void setMessage(const mesh_msgs::TriangleMeshStamped::ConstPtr& meshMsg);


   /**
     * @brief Sets the pose of the coordinate frame the message refers to.
     *
     * @param position The pose of the coordinate frame
     */
    void setFramePosition(const Ogre::Vector3& position);

    /**
     * @brief Sets the orientation of the coordinate frame the message refers to.
     *
     * @param orientation The orientation of the coordinate frame
     */
    void setFrameOrientation(const Ogre::Quaternion& orientation);

    /**
     * @brief Updates the visible parts of the mesh depending on input from the rviz display.
     *
     * @param showWireframe         When TRUE wireframe is visible
     * @param wireframeColor        The color of the wireframe
     * @param wireframeAlpha        The transparency of the wireframe
     * @param showFaces             When TRUE faces are visible
     * @param facesColor            The color of the faces
     * @param facesAlpha            The transparency of the faces
     * @param useVertexColors       When TRUE vertex colors are used
     * @param useTriangleColors     When TRUE triangle colors are used
     * @param showTextures          When TRUE textures are visible
     * @param showNormals           When TRUE normals are visible
     * @param normalsColor          The color of the normals
     * @param normalsAlpha          The transparency of the normals
     * @param normalsScallingFactor The size of the normals
     */
    void updateMaterial(bool showWireframe,
      Ogre::ColourValue wireframeColor,
      float wireframeAlpha,
      bool showFaces,
      Ogre::ColourValue facesColor,
      float facesAlpha,
      bool useVertexColors,
      bool useTriangleColors,
      bool showTextures,
      bool showNormals,
      Ogre::ColourValue normalsColor,
      float normalsAlpha,
      float normalsScallingFactor);

    /**
     * @brief Updates the size of the normals dynamically.
     *
     * @param scallingFactor The factor the normals have to be scaled with
     */
    void updateNormals(float scallingFactor);

private:

    void showWireframe(
      Ogre::Pass* pass,
      Ogre::ColourValue wireframeColor,
      float wireframeAlpha);

    void showFaces(
      Ogre::Pass* pass,
      Ogre::ColourValue facesColor,
      float facesAlpha,
      bool useVertexColors);

    void showNormals(
      Ogre::Pass* pass,
      Ogre::ColourValue normalsColor,
      float normalsAlpha);

    void showTextures(Ogre::Pass* pass);

    void enteringGeneralTriangleMesh(const mesh_msgs::TriangleMesh& mesh);
    void enteringColoredTriangleMesh(const mesh_msgs::TriangleMesh& mesh);
    void enteringNormals(const mesh_msgs::TriangleMesh& mesh);

    bool m_vertex_normals_enabled;
    bool m_vertex_colors_enabled;
    bool m_triangle_colors_enabled;
    bool m_texture_coords_enabled;

    //! Ogre Scenenode
    Ogre::SceneNode* m_sceneNode;

    //! The context that contains the display information.
    rviz::DisplayContext* m_displayContext;

    //! First ID of the created mesh
    size_t m_prefix;

    //! Second ID of the created mesh
    size_t m_postfix;

    //! Random ID of the created mesh
    size_t m_random;

    //! The mesh-object to display
    Ogre::ManualObject* m_mesh;

    //! The material for the general mesh
    Ogre::MaterialPtr m_meshGeneralMaterial;

    //! The material for the colored triangle mesh
    Ogre::MaterialPtr m_meshColoredTrianglesMaterial;

    //! The material of the normals
    Ogre::MaterialPtr m_normalMaterial;

    //! Counter for textures
    size_t m_textureCounter;

    //! Is the mesh textured?
    bool m_hasTextures;

    //! Map of fake textures (size = 1x1 = only a color)
    std::map<size_t, Ogre::ColourValue> m_fakeTextures;

    //! Factor the normal-size is multiplied with.
    float m_normalsScalingFactor;
};
} // End namespace rviz_mesh_plugin

#endif
