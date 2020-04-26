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
 *  textured_mesh_display.cpp
 *
 *
 *  authors:
 *
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#include "textured_mesh_display.h"

#include <mesh_msgs/GetVertexColors.h>
#include <mesh_msgs/GetMaterials.h>
#include <mesh_msgs/GetGeometry.h>
#include <mesh_msgs/GetTexture.h>
#include <mesh_msgs/GetUUID.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/display_context.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

#include <ros/callback_queue.h>

namespace rviz_mesh_plugin
{

size_t TexturedMeshDisplay::displayCounter = 0;

TexturedMeshDisplay::TexturedMeshDisplay()
{
    m_displayID = displayCounter++;
    m_tfMeshFilter = 0;
    m_meshCounter = 0;

    // topic properties
    m_meshTopic = new rviz::RosTopicProperty(
        "Geometry Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<mesh_msgs::MeshGeometryStamped>()),
        "Geometry topic to subscribe to.",
        this,
        SLOT(updateTopic())
    );

    // mesh buffer size property
    m_meshBufferSize = new rviz::IntProperty(
        "Mesh Buffer Size",
        1,
        "Number of prior meshes to display.",
        this,
        SLOT(updateMeshBufferSize())
    );
    m_meshBufferSize->setMin(1);

    // Display type selection dropdown
    m_displayType = new rviz::EnumProperty(
        "Display Type",
        "Faces with fixed color",
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

    m_costColorType = new rviz::EnumProperty(
        "Color Scale",
        "Rainbow",
        "Select color scale for vertex costs. Mesh will update when new data arrives.",
        m_displayType,
        SLOT(updateMesh()),
        this
    );
    m_costColorType->addOption("Rainbow", 0);
    m_costColorType->addOption("Red Green", 1);

    m_vertexColorsTopic = new rviz::RosTopicProperty(
        "Vertex Colors Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<mesh_msgs::MeshVertexColorsStamped>()),
        "Vertex color topic to subscribe to.",
        m_displayType,
        SLOT(updateTopic()),
        this
    );

    m_vertexCostsTopic = new rviz::RosTopicProperty(
        "Vertex Costs Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<mesh_msgs::MeshVertexCostsStamped>()),
        "Vertex cost topic to subscribe to.",
        m_displayType,
        SLOT(updateTopic()),
        this
    );

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

    // Vertex color service name property
    m_vertexColorServiceName = new rviz::StringProperty(
        "Vertex Color Service Name",
        "get_vertex_colors",
        "Name of the Vertex Color Service to request Vertex Colors from.",
        m_displayType,
        SLOT(updateVertexColorService()),
        this
    );

    // Material service name property
    m_materialServiceName = new rviz::StringProperty(
        "Material Service Name",
        "get_materials",
        "Name of the Matrial Service to request Materials from.",
        m_displayType,
        SLOT(updateMaterialAndTextureServices()),
        this
    );

    // Textures service name property
    m_textureServiceName = new rviz::StringProperty(
        "Texture Service Name",
        "get_texture",
        "Name of the Texture Service to request Textures from.",
        m_displayType,
        SLOT(updateMaterialAndTextureServices()),
        this
    );

    m_selectVertexCostMap = new rviz::EnumProperty(
        "Vertex Costs Type",
        "-- None --",
        "Select the type of vertex cost map to be displayed. New types will appear here when a new message arrives.",
        m_displayType,
        SLOT(updateVertexCosts()),
        this
    );
    m_selectVertexCostMap->addOption("-- None --", 0);


    m_showWireframe = new rviz::BoolProperty(
        "Show Wireframe",
        true,
        "Show Wireframe",
        this,
        SLOT(updateMesh()),
        this
    );

    // wireframe color property
    m_wireframeColor = new rviz::ColorProperty(
        "Wireframe Color",
        QColor(0, 0, 0),
        "The color of the wireframe.",
        m_showWireframe,
        SLOT(updateMesh()),
        this
    );
    // wireframe alpha property
    m_wireframeAlpha = new rviz::FloatProperty(
        "Wireframe Alpha",
        1.0,
        "The alpha-value of the wireframe",
        m_showWireframe,
        SLOT(updateMesh()),
        this
    );
    m_wireframeAlpha->setMin(0);
    m_wireframeAlpha->setMax(1);


    m_showNormals = new rviz::BoolProperty(
        "Show Normals",
        true,
        "Show Normals",
        this,
        SLOT(updateMesh()),
        this
    );

    m_normalsColor = new rviz::ColorProperty(
        "Normals Color",
        QColor(255, 0, 255),
        "The color of the normals.",
        m_showNormals,
        SLOT(updateMesh()),
        this
    );
    m_normalsAlpha = new rviz::FloatProperty(
        "Normals Alpha",
        1.0,
        "The alpha-value of the normals",
        m_showNormals,
        SLOT(updateMesh()),
        this
    );
    m_normalsAlpha->setMin(0);
    m_normalsAlpha->setMax(1);
    m_scalingFactor = new rviz::FloatProperty(
        "Normals Scaling Factor",
        0.1,
        "Scaling factor of the normals",
        m_showNormals,
        SLOT(updateMesh()),
        this
    );

    m_costUseCustomLimits = new rviz::BoolProperty(
        "Use Custom limits",
        false,
        "Use custom vertex cost limits",
        m_displayType,
        SLOT(updateVertexCosts()),
        this
    );

    m_costLowerLimit = new rviz::FloatProperty(
        "Vertex Costs Lower Limit",
        0.0,
        "Vertex costs lower limit",
        m_costUseCustomLimits,
        SLOT(updateVertexCosts()),
        this
    );
    m_costLowerLimit->hide();

    m_costUpperLimit = new rviz::FloatProperty(
        "Vertex Costs Upper Limit",
        1.0,
        "Vertex costs upper limit",
        m_costUseCustomLimits,
        SLOT(updateVertexCosts()),
        this
    );
    m_costUpperLimit->hide();

}

TexturedMeshDisplay::~TexturedMeshDisplay()
{
    unsubscribe();
    delete m_tfMeshFilter;
}

void TexturedMeshDisplay::onInitialize()
{
    m_tfMeshFilter = new tf2_ros::MessageFilter<mesh_msgs::MeshGeometryStamped>(
        *rviz::Display::context_->getTF2BufferPtr(),
        rviz::Display::fixed_frame_.toStdString(),
        2,
        rviz::Display::update_nh_
    );
    m_tfMeshFilter->connectInput(m_meshSubscriber);
    context_->getFrameManager()->registerFilterForTransformStatusCheck(m_tfMeshFilter, this);

    m_tfVertexColorsFilter = new tf2_ros::MessageFilter<mesh_msgs::MeshVertexColorsStamped>(
        *rviz::Display::context_->getTF2BufferPtr(),
        rviz::Display::fixed_frame_.toStdString(),
        10,
        rviz::Display::update_nh_
    );
    m_tfVertexColorsFilter->connectInput(m_vertexColorsSubscriber);
    context_->getFrameManager()->registerFilterForTransformStatusCheck(m_tfVertexColorsFilter, this);

    m_tfVertexCostsFilter = new tf2_ros::MessageFilter<mesh_msgs::MeshVertexCostsStamped>(
        *rviz::Display::context_->getTF2BufferPtr(),
        rviz::Display::fixed_frame_.toStdString(),
        10,
        rviz::Display::update_nh_
    );
    m_tfVertexCostsFilter->connectInput(m_vertexCostsSubscriber);
    context_->getFrameManager()->registerFilterForTransformStatusCheck(m_tfVertexCostsFilter, this);

    m_meshSynchronizer = 0;
    m_colorsSynchronizer = 0;
    m_costsSynchronizer = 0;

    updateMeshBufferSize();
    updateTopic();
    initServices();
    updateMesh();
}

void TexturedMeshDisplay::initialServiceCall()
{
    ros::NodeHandle n;
    ros::ServiceClient m_uuidClient = n.serviceClient<mesh_msgs::GetUUID>("get_uuid");

    mesh_msgs::GetUUID srv_uuid;
    if (m_uuidClient.call(srv_uuid))
    {
        std::string uuid = (std::string)srv_uuid.response.uuid;

        ROS_INFO_STREAM("Initial data available for UUID=" << uuid);

        ros::ServiceClient m_geometryClient = n.serviceClient<mesh_msgs::GetGeometry>("get_geometry");

        mesh_msgs::GetGeometry srv_geometry;
        srv_geometry.request.uuid = uuid;
        if (m_geometryClient.call(srv_geometry))
        {
            ROS_INFO_STREAM("Found geometry for UUID=" << uuid);
            mesh_msgs::MeshGeometryStamped::ConstPtr geometry
                = boost::make_shared<const mesh_msgs::MeshGeometryStamped>(srv_geometry.response.mesh_geometry_stamped);
            processMessage(geometry);
        }
        else
        {
            ROS_INFO_STREAM("Could not load geometry. Waiting for callback to trigger ... ");
        }
    }
    else
    {
        ROS_INFO("No initial data available, waiting for callback to trigger ...");
    }
}

void TexturedMeshDisplay::reset()
{
    rviz::Display::reset(); // TODO bad hack?!
    m_tfMeshFilter->clear();
    m_messagesReceived = 0;
    m_meshVisuals.clear();
}

void TexturedMeshDisplay::updateTopic()
{
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
}

void TexturedMeshDisplay::subscribe()
{
    if (!isEnabled())
    {
        return;
    }

    try {
        m_meshSubscriber.subscribe(update_nh_, m_meshTopic->getTopicStd(), 1);
        m_vertexColorsSubscriber.subscribe(update_nh_, m_vertexColorsTopic->getTopicStd(), 1);
        m_vertexCostsSubscriber.subscribe(update_nh_, m_vertexCostsTopic->getTopicStd(), 4);
        setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
    }
    catch(ros::Exception& e)
    {
        setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }

    // Nothing
    if (m_meshTopic->getTopicStd().empty())
    {
        m_meshBufferSize->hide();
        return;
    }
    else
    {
        m_meshBufferSize->show();
        m_meshSynchronizer =
            new message_filters::Cache<mesh_msgs::MeshGeometryStamped>(
                m_meshSubscriber, 10
            );
        m_meshSynchronizer->registerCallback(
            boost::bind(&TexturedMeshDisplay::incomingGeometry, this, _1)
        );

        m_colorsSynchronizer =
            new message_filters::Cache<mesh_msgs::MeshVertexColorsStamped>(
                m_vertexColorsSubscriber, 1
            );
        m_colorsSynchronizer->registerCallback(
            boost::bind(&TexturedMeshDisplay::incomingVertexColors, this, _1)
        );

        m_costsSynchronizer =
            new message_filters::Cache<mesh_msgs::MeshVertexCostsStamped>(
                m_vertexCostsSubscriber, 1
            );
        m_costsSynchronizer->registerCallback(
            boost::bind(&TexturedMeshDisplay::incomingVertexCosts, this, _1)
        );

    }

    initialServiceCall();
}

void TexturedMeshDisplay::unsubscribe()
{
    m_meshSubscriber.unsubscribe();
    m_vertexColorsSubscriber.unsubscribe();
    m_vertexCostsSubscriber.unsubscribe();

    if (m_meshSynchronizer)
    {
        delete m_meshSynchronizer;
        m_meshSynchronizer = 0;
    }
    if (m_colorsSynchronizer)
    {
        delete m_colorsSynchronizer;
        m_colorsSynchronizer = 0;
    }
    if (m_costsSynchronizer)
    {
        delete m_costsSynchronizer;
        m_costsSynchronizer = 0;
    }
}

void TexturedMeshDisplay::onEnable()
{
    subscribe();
}

void TexturedMeshDisplay::onDisable()
{
    unsubscribe();
    reset();
}

void TexturedMeshDisplay::fixedFrameChanged()
{
    m_tfMeshFilter->setTargetFrame(fixed_frame_.toStdString());
    reset();
}

void TexturedMeshDisplay::incomingVertexColors(const mesh_msgs::MeshVertexColorsStamped::ConstPtr& colorsStamped)
{
    if (m_meshVisuals.empty())
    {
        ROS_ERROR("Received vertex colors, but no visual available!");
        return;
    }
    if (colorsStamped->uuid.compare(m_lastUuid) != 0)
    {
        ROS_ERROR("Received vertex colors, but UUIDs dont match!");
        return;
    }

    getCurrentVisual()->setVertexColors(colorsStamped);

    updateMesh();
}

void TexturedMeshDisplay::incomingVertexCosts(const mesh_msgs::MeshVertexCostsStamped::ConstPtr& costsStamped)
{
    if (m_meshVisuals.empty())
    {
        ROS_ERROR("Received vertex costs, but no visual available!");
        return;
    }
    if (costsStamped->uuid.compare(m_lastUuid) != 0)
    {
        ROS_ERROR("Received vertex costs, but UUIDs dont match!");
        return;
    }

    cacheVertexCosts(costsStamped);
    updateVertexCosts();
}

void TexturedMeshDisplay::cacheVertexCosts(
    const mesh_msgs::MeshVertexCostsStamped::ConstPtr costsStamped
)
{
    ROS_INFO_STREAM("Cache vertex cost map '" << costsStamped->type << "' for UUID " << costsStamped->uuid);

    // insert into cache
    std::pair<std::map<std::string, const mesh_msgs::MeshVertexCostsStamped::ConstPtr>::iterator, bool> ret =
        m_costCache.insert(
            std::pair<std::string, const mesh_msgs::MeshVertexCostsStamped::ConstPtr>(costsStamped->type, costsStamped));
    if(ret.second)
    {
        ROS_INFO_STREAM("The cost layer \"" << costsStamped->type << "\" has been added.");
        m_selectVertexCostMap->addOptionStd(costsStamped->type, m_selectVertexCostMap->numChildren());
    }
    else
    {
        //m_selectVertexCostMap->addOptionStd(costsStamped->type, m_selectVertexCostMap->numChildren());
        m_costCache.erase(ret.first);
        m_costCache.insert(
            std::pair<std::string, const mesh_msgs::MeshVertexCostsStamped::ConstPtr>(costsStamped->type, costsStamped));
        ROS_INFO_STREAM("The cost layer \"" << costsStamped->type << "\" has been updated.");
    }

}

void TexturedMeshDisplay::incomingGeometry(const mesh_msgs::MeshGeometryStamped::ConstPtr& meshMsg)
{
    m_messagesReceived++;
    setStatus(rviz::StatusProperty::Ok, "Topic", QString::number(m_messagesReceived) + " messages received");
    processMessage(meshMsg);
}

void TexturedMeshDisplay::updateMeshBufferSize()
{
    reset();
    m_meshVisuals.rset_capacity(m_meshBufferSize->getInt());
}

void TexturedMeshDisplay::updateVertexCosts()
{
    if (m_costUseCustomLimits->getBool())
    {
        model_->expandProperty(m_costUseCustomLimits);
        m_costLowerLimit->show();
        m_costUpperLimit->show();

        if (m_meshVisuals.size() > 0 && m_costCache.count(m_selectVertexCostMap->getStdString()) != 0)
        {
            getCurrentVisual()->setVertexCosts(
                m_costCache[m_selectVertexCostMap->getStdString()],
                m_costColorType->getOptionInt(),
                m_costLowerLimit->getFloat(),
                m_costUpperLimit->getFloat()
            );
        }
    }
    else
    {
        m_costLowerLimit->hide();
        m_costUpperLimit->hide();
        if (m_meshVisuals.size() > 0 && m_costCache.count(m_selectVertexCostMap->getStdString()) != 0)
        {
            getCurrentVisual()->setVertexCosts(
                m_costCache[m_selectVertexCostMap->getStdString()],
                m_costColorType->getOptionInt()
            );
        }
    }
    updateMesh();
}


void TexturedMeshDisplay::updateMesh()
{
    bool showWireframe = m_showWireframe->getBool();
    bool showNormals = m_showNormals->getBool();
    bool showFaces = false;
    bool showTextures = false;
    bool useVertexColors = false;
    bool showVertexCosts = false;

    m_showTexturedFacesOnly->hide();
    m_facesColor->hide();
    m_facesAlpha->hide();
    m_vertexColorServiceName->hide();
    m_materialServiceName->hide();
    m_textureServiceName->hide();
    m_vertexColorsTopic->hide();
    m_selectVertexCostMap->hide();
    m_vertexCostsTopic->hide();
    m_costColorType->hide();
    m_costUseCustomLimits->hide();
    deleteStatus("Services");

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
            useVertexColors = true;
            m_vertexColorServiceName->show();
            m_vertexColorsTopic->show();
            updateVertexColorService();
            break;
        case 2: // Faces with textures
            showFaces = true;
            showTextures = true;
            m_showTexturedFacesOnly->show();
            m_materialServiceName->show();
            m_textureServiceName->show();
            updateMaterialAndTextureServices();
            break;
        case 3: // Faces with vertex costs
            showFaces = true;
            showVertexCosts = true;
            m_selectVertexCostMap->show();
            m_vertexCostsTopic->show();
            m_costColorType->show();
            m_costUseCustomLimits->show();
            break;
        case 4: // No Faces
            break;
    }

    if (!m_meshVisuals.empty())
    {
        for (
            boost::circular_buffer<boost::shared_ptr<TexturedMeshVisual> >::iterator it = m_meshVisuals.begin();
            it != m_meshVisuals.end();
            it++
        )
        {
            it->get()->updateMaterial(
                showWireframe, m_wireframeColor->getOgreColor(), m_wireframeAlpha->getFloat(),
                showFaces, m_facesColor->getOgreColor(), m_facesAlpha->getFloat(), useVertexColors, showVertexCosts,
                showTextures, m_showTexturedFacesOnly->getBool(),
                showNormals, m_normalsColor->getOgreColor(), m_normalsAlpha->getFloat(), m_scalingFactor->getFloat()
            );
        }
    }
}

void TexturedMeshDisplay::initServices()
{
    // Initialize service clients
    ros::NodeHandle n;
    m_vertexColorClient = n.serviceClient<mesh_msgs::GetVertexColors>(m_vertexColorServiceName->getStdString());

    m_materialsClient = n.serviceClient<mesh_msgs::GetMaterials>(m_materialServiceName->getStdString());

    m_textureClient = n.serviceClient<mesh_msgs::GetTexture>(m_textureServiceName->getStdString());
}

void TexturedMeshDisplay::updateVertexColorService()
{
    // Check if the service name is valid
    std::string error;
    if(!ros::names::validate(m_vertexColorServiceName->getStdString(), error))
    {
        setStatus(rviz::StatusProperty::Warn, "Services", QString("The service name contains an invalid character."));
        return;
    }

    // Update vertex color service client
    ros::NodeHandle n;
    m_vertexColorClient = n.serviceClient<mesh_msgs::GetVertexColors>(m_vertexColorServiceName->getStdString());
    if (m_vertexColorClient.exists())
    {
        setStatus(rviz::StatusProperty::Ok, "Services", "Vertex Color Service OK");
        if (!m_meshVisuals.empty())
        {
            requestVertexColors(m_meshVisuals.back(), m_lastUuid);
        }
    }
    else
    {
        setStatus(rviz::StatusProperty::Warn, "Services", QString("The specified Vertex Color Service doesn't exist."));
    }
}

void TexturedMeshDisplay::updateMaterialAndTextureServices()
{
    // Check if the service names are valid
    std::string error;
    if(!ros::names::validate(m_materialServiceName->getStdString(), error)
        || !ros::names::validate(m_textureServiceName->getStdString(), error))
    {
        setStatus(rviz::StatusProperty::Warn, "Services", QString("The service name contains an invalid character."));
        return;
    }

    // Update material and texture service clients
    ros::NodeHandle n;
    m_materialsClient = n.serviceClient<mesh_msgs::GetMaterials>(m_materialServiceName->getStdString());
    m_textureClient = n.serviceClient<mesh_msgs::GetTexture>(m_textureServiceName->getStdString());
    if (m_materialsClient.exists())
    {
        if (!m_meshVisuals.empty())
        {
            requestMaterials(m_meshVisuals.back(), m_lastUuid);
        }
        if (m_textureClient.exists())
        {
            setStatus(rviz::StatusProperty::Ok, "Services", "Material and Texture Service OK");
        }
        else
        {
            setStatus(rviz::StatusProperty::Warn, "Services", QString("The specified Texture Service doesn't exist."));
        }
    }
    else
    {
        setStatus(rviz::StatusProperty::Warn, "Services", QString("The specified Material Service doesn't exist."));
    }
}

boost::shared_ptr<TexturedMeshVisual> TexturedMeshDisplay::getNewVisual()
{
    boost::shared_ptr<TexturedMeshVisual> visual;
    if (m_meshVisuals.full())
    {
        visual = m_meshVisuals.front();
        m_meshVisuals.push_back(visual);
    }
    else
    {
        int randomId = (int)((double)rand() / RAND_MAX * 9998);

        visual.reset(new TexturedMeshVisual(context_, m_displayID, m_meshCounter, randomId));

        m_meshVisuals.push_back(visual);
        m_meshCounter++;
    }
    return visual;
}

boost::shared_ptr<TexturedMeshVisual> TexturedMeshDisplay::getCurrentVisual()
{

    if (m_meshVisuals.empty())
    {
        ROS_ERROR("Requested current visual when none is available!");
    }

    return m_meshVisuals.back();
}

void TexturedMeshDisplay::processMessage(const mesh_msgs::MeshGeometryStamped::ConstPtr& meshMsg)
{
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (!context_->getFrameManager()->getTransform(
        meshMsg->header.frame_id,
        meshMsg->header.stamp,
        position, orientation)
    )
    {
        ROS_ERROR(
            "Error transforming from frame '%s' to frame '%s'",
            meshMsg->header.frame_id.c_str(), qPrintable(rviz::Display::fixed_frame_)
        );
        return;
    }

    if (!m_lastUuid.empty() && meshMsg->uuid.compare(m_lastUuid) != 0)
    {
        ROS_WARN("Received geometry with new UUID!");
        m_costCache.clear();

    }

    m_lastUuid = meshMsg->uuid;

    boost::shared_ptr<TexturedMeshVisual> visual = getNewVisual();

    visual->setGeometry(meshMsg);
    requestVertexColors(visual, meshMsg->uuid);
    requestMaterials(visual, meshMsg->uuid);
    updateMesh();
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);
}

void TexturedMeshDisplay::requestVertexColors(boost::shared_ptr<TexturedMeshVisual> visual, std::string uuid)
{
    mesh_msgs::GetVertexColors srv;
    srv.request.uuid = uuid;
    if (m_vertexColorClient.call(srv))
    {
        ROS_INFO("Successful vertex colors service call!");
        mesh_msgs::MeshVertexColorsStamped::ConstPtr meshVertexColors =
            boost::make_shared<const mesh_msgs::MeshVertexColorsStamped>(srv.response.mesh_vertex_colors_stamped);

        visual->setVertexColors(meshVertexColors);
    }
    else
    {
        ROS_INFO("Failed vertex colors service call!");
    }
}

void TexturedMeshDisplay::requestMaterials(boost::shared_ptr<TexturedMeshVisual> visual, std::string uuid)
{
    mesh_msgs::GetMaterials srv;
    srv.request.uuid = uuid;
    if (m_materialsClient.call(srv))
    {
        ROS_INFO("Successful materials service call!");

        mesh_msgs::MeshMaterialsStamped::ConstPtr meshMaterialsStamped =
            boost::make_shared<const mesh_msgs::MeshMaterialsStamped>(srv.response.mesh_materials_stamped);


        visual->setMaterials(meshMaterialsStamped);

        for (mesh_msgs::MeshMaterial material : meshMaterialsStamped->mesh_materials.materials)
        {
            if (material.has_texture)
            {
                mesh_msgs::GetTexture texSrv;
                texSrv.request.uuid = uuid;
                texSrv.request.texture_index = material.texture_index;
                if (m_textureClient.call(texSrv))
                {
                    ROS_INFO("Successful texture service call with index %d!", material.texture_index);
                    mesh_msgs::MeshTexture::ConstPtr texture =
                        boost::make_shared<const mesh_msgs::MeshTexture>(texSrv.response.texture);

                    visual->addTexture(texture);
                }
                else
                {
                    ROS_INFO("Failed texture service call with index %d!", material.texture_index);
                }
            }
        }
    }
    else
    {
        ROS_INFO("Failed materials service call!");
    }
}

} // end namespace rviz_mesh_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_mesh_plugin::TexturedMeshDisplay, rviz::Display)
