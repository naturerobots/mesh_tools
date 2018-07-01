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
 *  MeshDisplay.hpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */


#ifndef MESH_DISPLAY_HPP
#define MESH_DISPLAY_HPP

#include <Types.hpp>
#include <TexturedMeshVisual.hpp>

#include <vector>
#include <memory>

#include <string>
#include <math.h>
#include <algorithm>

#include <QMessageBox>
#include <QApplication>
#include <QIcon>

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/visualization_frame.h>
#include <rviz/geometry.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <rviz/mesh_loader.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreStringConverter.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreColourValue.h>

#endif

namespace rviz
{

// Forward declaration
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class StringProperty;

} // End namespace rviz

namespace rviz_map_plugin
{

using std::shared_ptr;
using std::unique_ptr;
using std::string;
using std::vector;

// Forward declaration
class TexturedMeshVisual;


/**
 * @class MeshDisplay
 * @brief Display for showing the mesh in different modes
 */
class MeshDisplay: public rviz::Display
{
Q_OBJECT

public:

    /**
     * @brief Constructor
     */
    MeshDisplay();

    /**
     * @brief Destructor
     */
    ~MeshDisplay();

    /**
     * @brief Set the geometry
     * @param geometry The geometry
     */
    void setGeometry(shared_ptr<Geometry> geometry);

    /**
     * @brief Set the vertex colors
     * @param vertexColors The vertex colors
     */
    void setVertexColors(vector<Color>& vertexColors);

    /**
     * @brief Set the vertex normals
     * @param vertexNormals The vertex normals
     */
    void setVertexNormals(vector<Normal>& vertexNormals);

    /**
     * @brief Set the materials and texture coordinates
     * @param materials The materials
     * @param texCoords The texture coordinates
     */
    void setMaterials(vector<Material>& materials, vector<TexCoords>& texCoords);

    /**
     * @brief Add a texture
     * @param texture The texture
     * @param textureIndex The textures index
     */
    void addTexture(Texture& texture, uint32_t textureIndex);

    /**
     * @brief RViz callback on enable
     */
    void onEnable();

    /**
     * @brief RViz callback on disable
     */
    void onDisable();

private Q_SLOTS:

    /**
     * @brief Updates the mesh
     */
    void updateMesh();

    /**
     * @brief Updates the mesh wireframe
     */
    void updateWireframe();

    /**
     * @brief Update the mesh normals
     */
    void updateNormals();

private:

    /**
     * @brief RViz callback on initialize
     */
    void onInitialize();

    /// Geometry data
    shared_ptr<Geometry> m_geometry;

    /// Visual data
    shared_ptr<TexturedMeshVisual> m_visual;

    /// Property to set wireframe color
    rviz::ColorProperty* m_wireframeColor;

    /// Property to set wireframe transparency
    rviz::FloatProperty* m_wireframeAlpha;

    /// Property to set faces color
    rviz::ColorProperty* m_facesColor;

    /// Property to set faces transparency
    rviz::FloatProperty* m_facesAlpha;

    /// Property to use the vertex colors
    rviz::BoolProperty* m_facesVertexColors;

    /// Property to use the triangle colors
    rviz::BoolProperty* m_facesTriangleColors;

    /// Property to set the size of the normals
    rviz::FloatProperty* m_scalingFactor;

    /// Property to set the color of the normals
    rviz::ColorProperty* m_normalsColor;

    /// Property to set the transparency of the normals
    rviz::FloatProperty* m_normalsAlpha;

    /// Property to select the display type
    rviz::EnumProperty* m_displayType;

    /// Property to select the wireframe
    rviz::BoolProperty* m_showWireframe;

    /// Property to select the normals
    rviz::BoolProperty* m_showNormals;

    /// Property to only show textured faces when texturizing is enabled
    rviz::BoolProperty* m_showTexturedFacesOnly;

    /// Will be set to true once the initial data has arrived
    bool has_data = false;

};

} // end namespace rviz_map_plugin

#endif
