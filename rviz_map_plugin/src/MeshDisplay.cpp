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
 *  MeshDisplay.cpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#include <MeshDisplay.hpp>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>


namespace rviz_map_plugin
{

MeshDisplay::MeshDisplay(): rviz::Display()
{

    // Display type selection dropdown
    m_displayType = new rviz::EnumProperty(
        "Display Type",
        "Fixed Color",
        "Select Display Type for Mesh",
        this,
        SLOT(updateMesh()),
        this
    );
    m_displayType->addOption("Fixed Color", 0);
    m_displayType->addOption("Vertex Color", 1);
    m_displayType->addOption("Textures", 2);
    m_displayType->addOption("Vertex Costs", 3);
    m_displayType->addOption("Hide Faces", 4);

    m_showTexturedFacesOnly = new rviz::BoolProperty(
        "Show textured faces only",
        false,
        "Show textured faces only",
        m_displayType,
        SLOT(updateMesh()),
        this
    );

     // face color properties
    m_facesColor = new rviz::ColorProperty(
        "Faces Color",
        QColor(0, 255, 0),
        "The color of the faces.",
        m_displayType,
        SLOT(updateMesh()),
        this
    );

    // face alpha properties
    m_facesAlpha = new rviz::FloatProperty(
        "Faces Alpha",
        1.0,
        "The alpha-value of the faces",
        m_displayType,
        SLOT(updateMesh()),
        this
    );
    m_facesAlpha->setMin(0);
    m_facesAlpha->setMax(1);


    m_showWireframe = new rviz::BoolProperty(
        "Show Wireframe",
        true,
        "Show Wireframe",
        this,
        SLOT(updateWireframe()),
        this
    );

    // wireframe color property
    m_wireframeColor = new rviz::ColorProperty(
        "Wireframe Color",
        QColor(0, 0, 0),
        "The color of the wireframe.",
        m_showWireframe,
        SLOT(updateWireframe()),
        this
    );
    // wireframe alpha property
    m_wireframeAlpha = new rviz::FloatProperty(
        "Wireframe Alpha",
        1.0,
        "The alpha-value of the wireframe",
        m_showWireframe,
        SLOT(updateWireframe()),
        this
    );
    m_wireframeAlpha->setMin(0);
    m_wireframeAlpha->setMax(1);


    m_showNormals = new rviz::BoolProperty(
        "Show Normals",
        true,
        "Show Normals",
        this,
        SLOT(updateNormals()),
        this
    );

    m_normalsColor = new rviz::ColorProperty(
        "Normals Color",
        QColor(255, 0, 255),
        "The color of the normals.",
        m_showNormals,
        SLOT(updateNormals()),
        this
    );
    m_normalsAlpha = new rviz::FloatProperty(
        "Normals Alpha",
        1.0,
        "The alpha-value of the normals",
        m_showNormals,
        SLOT(updateNormals()),
        this
    );
    m_normalsAlpha->setMin(0);
    m_normalsAlpha->setMax(1);
    m_scalingFactor = new rviz::FloatProperty(
        "Normals Scaling Factor",
        0.1,
        "Scaling factor of the normals",
        m_showNormals,
        SLOT(updateNormals()),
        this
    );

    setStatus(rviz::StatusProperty::Error, "Display", "Can't be used without Map3D plugin or no data is available");
}

MeshDisplay::~MeshDisplay()
{
}


// =====================================================================================================================
// Callbacks

void MeshDisplay::onInitialize()
{
    updateMesh();
    updateWireframe();
    updateNormals();
}

void MeshDisplay::onEnable()
{
    // Create the visual
    updateMesh();
    updateWireframe();
    updateNormals();
}

void MeshDisplay::onDisable()
{
    if (m_visual)
    {
        m_visual->updateMaterial(
            false,
            Ogre::ColourValue(),
            0.0f,
            false,
            false,
            false,
            false
        );
        m_visual->updateWireframe(false, Ogre::ColourValue(), 0.0f);
        m_visual->updateNormals(
            false,
            Ogre::ColourValue(),
            0.0f,
            1.0f
        );
    }
}

// =====================================================================================================================
// Callbacks triggered from UI events (mostly)

void MeshDisplay::updateMesh()
{
    ROS_INFO("Mesh Display: Update");

    bool showFaces = false;
    bool showTextures = false;
    bool showVertexColors = false;
    bool showVertexCosts = false;

    m_showTexturedFacesOnly->hide();
    m_facesColor->hide();
    m_facesAlpha->hide();

    switch (m_displayType->getOptionInt())
    {
        default:
        case 0: // Faces with fixed color
            showFaces = true;
            m_facesColor->show();
            m_facesAlpha->show();
            break;
        case 1: // Faces with vertex color
            showFaces = true;
            showVertexColors = true;
            break;
        case 2: // Faces with textures
            showFaces = true;
            showTextures = true;
            m_showTexturedFacesOnly->show();
            break;
        case 3: // Faces with vertex costs
            showFaces = true;
            showVertexCosts = true;
            break;
        case 4: // No Faces
            break;
    }

    if (!has_data)
    {
        ROS_ERROR("Mesh display: no data available, can't draw mesh!");
        return;
    }

    if (isEnabled())
    {
        m_visual->updateMaterial(
            showFaces,
            m_facesColor->getOgreColor(),
            m_facesAlpha->getFloat(),
            showVertexColors,
            showVertexCosts,
            showTextures,
            m_showTexturedFacesOnly->getBool()
        );
    }

}

void MeshDisplay::updateWireframe()
{
    bool showWireframe = m_showWireframe->getBool();

    if (m_visual)
    {
        m_visual->updateWireframe(showWireframe, m_wireframeColor->getOgreColor(), m_wireframeAlpha->getFloat());
    }

}

void MeshDisplay::updateNormals()
{
    bool showNormals = m_showNormals->getBool();

    if (m_visual)
    {
        m_visual->updateNormals(
            showNormals,
            m_normalsColor->getOgreColor(),
            m_normalsAlpha->getFloat(),
            m_scalingFactor->getFloat()
        );
    }

}


// =====================================================================================================================
// Data loading

void MeshDisplay::setGeometry(shared_ptr<Geometry> geometry)
{
    m_geometry = geometry;

    // Create the visual
    int randomId = (int)((double)rand() / RAND_MAX * 9998);
    m_visual.reset(new TexturedMeshVisual(context_, 0, 0, randomId));

    m_visual->setGeometry(*geometry);
    has_data = true;
    if (isEnabled())
    {
        updateMesh();
        updateNormals();
        updateWireframe();
    }
    setStatus(rviz::StatusProperty::Ok, "Display", "");
}

void MeshDisplay::setVertexColors(vector<Color>& vertexColors)
{
    if (has_data)
    {
        m_visual->setVertexColors(vertexColors);
    }
    updateMesh();
}

void MeshDisplay::setVertexNormals(vector<Normal>& vertexNormals)
{
    if (has_data)
    {
        m_visual->setNormals(vertexNormals);
    }
    if (isEnabled())
    {
        updateNormals();
    }
}

void MeshDisplay::setMaterials(vector<Material>& materials, vector<TexCoords>& texCoords)
{
    if (has_data)
    {
        m_visual->setMaterials(materials, texCoords);
    }
    updateMesh();
}

void MeshDisplay::addTexture(Texture& texture, uint32_t textureIndex)
{
    if (has_data)
    {
        m_visual->addTexture(texture, textureIndex);
    }
}

} // End namespace rviz_map_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_map_plugin::MeshDisplay, rviz::Display)
