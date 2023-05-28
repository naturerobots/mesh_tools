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
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 *    Malte kleine Piening <malte@klpiening.de>
 */

#include <MeshDisplay.hpp>

#include <mesh_msgs/GetVertexColors.h>
#include <mesh_msgs/GetMaterials.h>
#include <mesh_msgs/GetGeometry.h>
#include <mesh_msgs/GetTexture.h>
#include <mesh_msgs/GetUUIDs.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>

namespace rviz_map_plugin
{
MeshDisplay::MeshDisplay() : rviz::Display(), m_ignoreMsgs(false)
{
  // mesh topic
  m_meshTopic = new rviz::RosTopicProperty(
      "Geometry Topic", "", QString::fromStdString(ros::message_traits::datatype<mesh_msgs::MeshGeometryStamped>()),
      "Geometry topic to subscribe to.", this, SLOT(updateTopic()));

  // buffer size / amount of meshes visualized
  m_bufferSize = new rviz::IntProperty("Buffer Size", 1, "Amount of meshes visualized", this, SLOT(updateBufferSize()));
  m_bufferSize->setMin(1);

  // Display Type
  {
    m_displayType = new rviz::EnumProperty("Display Type", "Fixed Color", "Select Display Type for Mesh", this,
                                           SLOT(updateMesh()), this);
    m_displayType->addOption("Fixed Color", 0);
    m_displayType->addOption("Vertex Color", 1);
    m_displayType->addOption("Textures", 2);
    m_displayType->addOption("Vertex Costs", 3);
    m_displayType->addOption("Hide Faces", 4);

    // Fixed Color
    {
      // face color properties
      m_facesColor = new rviz::ColorProperty("Faces Color", QColor(0, 255, 0), "The color of the faces.", m_displayType,
                                             SLOT(updateMesh()), this);

      // face alpha properties
      m_facesAlpha = new rviz::FloatProperty("Faces Alpha", 1.0, "The alpha-value of the faces", m_displayType,
                                             SLOT(updateMesh()), this);
      m_facesAlpha->setMin(0);
      m_facesAlpha->setMax(1);
    }

    // Vertex Color
    {
      m_vertexColorsTopic = new rviz::RosTopicProperty(
          "Vertex Colors Topic", "",
          QString::fromStdString(ros::message_traits::datatype<mesh_msgs::MeshVertexColorsStamped>()),
          "Vertex color topic to subscribe to.", m_displayType, SLOT(updateVertexColorsTopic()), this);

      m_vertexColorServiceName = new rviz::StringProperty("Vertex Color Service Name", "get_vertex_colors",
                                                          "Name of the Vertex Color Service to request Vertex Colors "
                                                          "from.",
                                                          m_displayType, SLOT(updateVertexColorService()), this);
    }

    // Textures
    {
      m_showTexturedFacesOnly = new rviz::BoolProperty("Show textured faces only", false, "Show textured faces only",
                                                       m_displayType, SLOT(updateMesh()), this);

      m_materialServiceName = new rviz::StringProperty("Material Service Name", "get_materials",
                                                       "Name of the Matrial Service to request Materials from.",
                                                       m_displayType, SLOT(updateMaterialAndTextureServices()), this);

      m_textureServiceName = new rviz::StringProperty("Texture Service Name", "get_texture",
                                                      "Name of the Texture Service to request Textures from.",
                                                      m_displayType, SLOT(updateMaterialAndTextureServices()), this);
    }

    // Vertex Costs
    {
      m_costColorType = new rviz::EnumProperty("Color Scale", "Rainbow",
                                               "Select color scale for vertex costs. Mesh will update when new data "
                                               "arrives.",
                                               m_displayType, SLOT(updateVertexCosts()), this);
      m_costColorType->addOption("Rainbow", 0);
      m_costColorType->addOption("Red Green", 1);

      m_vertexCostsTopic = new rviz::RosTopicProperty(
          "Vertex Costs Topic", "",
          QString::fromStdString(ros::message_traits::datatype<mesh_msgs::MeshVertexCostsStamped>()),
          "Vertex cost topic to subscribe to.", m_displayType, SLOT(updateVertexCostsTopic()), this);

      m_selectVertexCostMap = new rviz::EnumProperty("Vertex Costs Type", "-- None --",
                                                     "Select the type of vertex cost map to be displayed. New types "
                                                     "will appear here when a new message arrives.",
                                                     m_displayType, SLOT(updateVertexCosts()), this);
      m_selectVertexCostMap->addOption("-- None --", 0);

      m_costUseCustomLimits = new rviz::BoolProperty("Use Custom limits", false, "Use custom vertex cost limits",
                                                     m_displayType, SLOT(updateVertexCosts()), this);

      // custom cost limits
      {
        m_costLowerLimit = new rviz::FloatProperty("Vertex Costs Lower Limit", 0.0, "Vertex costs lower limit",
                                                   m_costUseCustomLimits, SLOT(updateVertexCosts()), this);
        m_costLowerLimit->hide();

        m_costUpperLimit = new rviz::FloatProperty("Vertex Costs Upper Limit", 1.0, "Vertex costs upper limit",
                                                   m_costUseCustomLimits, SLOT(updateVertexCosts()), this);
        m_costUpperLimit->hide();
      }
    }
  }

  // Wireframe
  {
    m_showWireframe =
        new rviz::BoolProperty("Show Wireframe", true, "Show Wireframe", this, SLOT(updateWireframe()), this);

    // wireframe color property
    m_wireframeColor = new rviz::ColorProperty("Wireframe Color", QColor(0, 0, 0), "The color of the wireframe.",
                                               m_showWireframe, SLOT(updateWireframe()), this);
    // wireframe alpha property
    m_wireframeAlpha = new rviz::FloatProperty("Wireframe Alpha", 1.0, "The alpha-value of the wireframe",
                                               m_showWireframe, SLOT(updateWireframe()), this);
    m_wireframeAlpha->setMin(0);
    m_wireframeAlpha->setMax(1);
  }

  // Normals
  {
    m_showNormals = new rviz::BoolProperty("Show Normals", true, "Show Normals", this, SLOT(updateNormals()), this);

    m_normalsColor = new rviz::ColorProperty("Normals Color", QColor(255, 0, 255), "The color of the normals.",
                                             m_showNormals, SLOT(updateNormalsColor()), this);
    m_normalsAlpha = new rviz::FloatProperty("Normals Alpha", 1.0, "The alpha-value of the normals", m_showNormals,
                                             SLOT(updateNormalsColor()), this);
    m_normalsAlpha->setMin(0);
    m_normalsAlpha->setMax(1);
    m_scalingFactor = new rviz::FloatProperty("Normals Scaling Factor", 0.1, "Scaling factor of the normals",
                                              m_showNormals, SLOT(updateNormalsSize()), this);
  }
}

MeshDisplay::~MeshDisplay()
{
}

void MeshDisplay::onInitialize()
{
  m_tfMeshFilter = new tf2_ros::MessageFilter<mesh_msgs::MeshGeometryStamped>(
      *rviz::Display::context_->getTF2BufferPtr(), rviz::Display::fixed_frame_.toStdString(), 2,
      rviz::Display::update_nh_);
  m_tfMeshFilter->connectInput(m_meshSubscriber);
  context_->getFrameManager()->registerFilterForTransformStatusCheck(m_tfMeshFilter, this);

  m_tfVertexColorsFilter = new tf2_ros::MessageFilter<mesh_msgs::MeshVertexColorsStamped>(
      *rviz::Display::context_->getTF2BufferPtr(), rviz::Display::fixed_frame_.toStdString(), 10,
      rviz::Display::update_nh_);
  m_tfVertexColorsFilter->connectInput(m_vertexColorsSubscriber);
  context_->getFrameManager()->registerFilterForTransformStatusCheck(m_tfVertexColorsFilter, this);

  m_tfVertexCostsFilter = new tf2_ros::MessageFilter<mesh_msgs::MeshVertexCostsStamped>(
      *rviz::Display::context_->getTF2BufferPtr(), rviz::Display::fixed_frame_.toStdString(), 10,
      rviz::Display::update_nh_);
  m_tfVertexCostsFilter->connectInput(m_vertexCostsSubscriber);
  context_->getFrameManager()->registerFilterForTransformStatusCheck(m_tfVertexCostsFilter, this);

  m_meshSynchronizer = 0;
  m_colorsSynchronizer = 0;
  m_costsSynchronizer = 0;

  // Initialize service clients
  ros::NodeHandle n;
  m_vertexColorClient = n.serviceClient<mesh_msgs::GetVertexColors>(m_vertexColorServiceName->getStdString());

  m_materialsClient = n.serviceClient<mesh_msgs::GetMaterials>(m_materialServiceName->getStdString());

  m_textureClient = n.serviceClient<mesh_msgs::GetTexture>(m_textureServiceName->getStdString());

  updateMesh();
  updateWireframe();
  updateNormals();
  updateTopic();
}

void MeshDisplay::onEnable()
{
  m_messagesReceived = 0;
  subscribe();
  updateMesh();
  updateWireframe();
  updateNormals();
}

void MeshDisplay::onDisable()
{
  unsubscribe();

  // clear visuals
  std::queue<std::shared_ptr<MeshVisual>>().swap(m_visuals);
}

void MeshDisplay::subscribe()
{
  if (!isEnabled() || m_ignoreMsgs)
  {
    return;
  }

  try
  {
    m_meshSubscriber.subscribe(update_nh_, m_meshTopic->getTopicStd(), 1);
    m_vertexColorsSubscriber.subscribe(update_nh_, m_vertexColorsTopic->getTopicStd(), 1);
    m_vertexCostsSubscriber.subscribe(update_nh_, m_vertexCostsTopic->getTopicStd(), 4);
    setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }

  // Nothing
  if (m_meshTopic->getTopicStd().empty())
  {
    return;
  }
  else
  {
    m_meshSynchronizer = new message_filters::Cache<mesh_msgs::MeshGeometryStamped>(m_meshSubscriber, 10);
    m_meshSynchronizer->registerCallback(boost::bind(&MeshDisplay::incomingGeometry, this, boost::placeholders::_1));

    m_colorsSynchronizer = new message_filters::Cache<mesh_msgs::MeshVertexColorsStamped>(m_vertexColorsSubscriber, 1);
    m_colorsSynchronizer->registerCallback(boost::bind(&MeshDisplay::incomingVertexColors, this, boost::placeholders::_1));

    m_costsSynchronizer = new message_filters::Cache<mesh_msgs::MeshVertexCostsStamped>(m_vertexCostsSubscriber, 1);
    m_costsSynchronizer->registerCallback(boost::bind(&MeshDisplay::incomingVertexCosts, this, boost::placeholders::_1));
  }

  initialServiceCall();
}

void MeshDisplay::unsubscribe()
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

void MeshDisplay::ignoreIncomingMessages()
{
  m_ignoreMsgs = true;
  unsubscribe();
  updateMesh();

  // only allow one mesh to be visualized
  while (m_visuals.size() > 1)
  {
    m_visuals.pop();
  }
}

// =====================================================================================================================
// Data manipulators

void MeshDisplay::setGeometry(shared_ptr<Geometry> geometry)
{
  // Create the visual
  std::shared_ptr<MeshVisual> visual = addNewVisual();
  visual->setGeometry(*geometry);
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
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->setVertexColors(vertexColors);
  }
  updateMesh();
}

void MeshDisplay::clearVertexCosts()
{
  m_costCache.clear();
  updateVertexCosts();
}

void MeshDisplay::addVertexCosts(std::string costlayer, std::vector<float>& vertexCosts)
{
  cacheVertexCosts(costlayer, vertexCosts);
  updateVertexCosts();
}

void MeshDisplay::setVertexNormals(vector<Normal>& vertexNormals)
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->setNormals(vertexNormals);
  }
  if (isEnabled())
  {
    updateNormals();
  }
}

void MeshDisplay::setMaterials(vector<Material>& materials, vector<TexCoords>& texCoords)
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->setMaterials(materials, texCoords);
  }
  updateMesh();
}

void MeshDisplay::addTexture(Texture& texture, uint32_t textureIndex)
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->addTexture(texture, textureIndex);
  }
}

void MeshDisplay::setPose(Ogre::Vector3& position, Ogre::Quaternion& orientation)
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);
  }
}

// =====================================================================================================================
// Callbacks triggered from UI events (mostly)

void MeshDisplay::updateBufferSize()
{
  while (m_visuals.size() > m_bufferSize->getInt())
  {
    m_visuals.pop();
  }
}

void MeshDisplay::updateMesh()
{
  ROS_INFO("Mesh Display: Update");

  bool showFaces = false;
  bool showTextures = false;
  bool showVertexColors = false;
  bool showVertexCosts = false;

  m_facesColor->hide();
  m_facesAlpha->hide();

  m_vertexColorsTopic->hide();
  m_vertexColorServiceName->hide();

  m_showTexturedFacesOnly->hide();
  m_materialServiceName->hide();
  m_textureServiceName->hide();

  m_costColorType->hide();
  m_vertexCostsTopic->hide();
  m_selectVertexCostMap->hide();
  m_costUseCustomLimits->hide();
  m_costLowerLimit->hide();
  m_costUpperLimit->hide();

  if (m_ignoreMsgs)
  {
    m_meshTopic->hide();
    m_bufferSize->hide();
  }
  else
  {
    m_meshTopic->show();
    m_bufferSize->show();
  }

  switch (m_displayType->getOptionInt())
  {
    default:
    case 0:  // Faces with fixed color
      showFaces = true;
      m_facesColor->show();
      m_facesAlpha->show();
      break;
    case 1:  // Faces with vertex color
      showFaces = true;
      showVertexColors = true;
      if (!m_ignoreMsgs)
      {
        m_vertexColorsTopic->show();
        m_vertexColorServiceName->show();
      }
      break;
    case 2:  // Faces with textures
      showFaces = true;
      showTextures = true;
      m_showTexturedFacesOnly->show();
      if (!m_ignoreMsgs)
      {
        m_materialServiceName->show();
        m_textureServiceName->show();
      }
      break;
    case 3:  // Faces with vertex costs
      showFaces = true;
      showVertexCosts = true;
      m_costColorType->show();
      if (!m_ignoreMsgs)
      {
        m_vertexCostsTopic->show();
      }
      m_selectVertexCostMap->show();
      m_costUseCustomLimits->show();
      if (m_costUseCustomLimits->getBool())
      {
        m_costLowerLimit->show();
        m_costUpperLimit->show();
      }
      break;
    case 4:  // No Faces
      break;
  }

  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (!visual)
  {
    ROS_ERROR("Mesh display: no visual available, can't draw mesh! (maybe no data has been received yet?)");
    return;
  }

  if (isEnabled())
  {
    visual->updateMaterial(showFaces, m_facesColor->getOgreColor(), m_facesAlpha->getFloat(), showVertexColors,
                           showVertexCosts, showTextures, m_showTexturedFacesOnly->getBool());
    updateWireframe();
  }
}

void MeshDisplay::updateWireframe()
{
  bool showWireframe = m_showWireframe->getBool();

  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->updateWireframe(showWireframe, m_wireframeColor->getOgreColor(), m_wireframeAlpha->getFloat());
  }
}

void MeshDisplay::updateNormals()
{
  bool showNormals = m_showNormals->getBool();

  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->updateNormals(showNormals, m_normalsColor->getOgreColor(), m_normalsAlpha->getFloat(),
                          m_scalingFactor->getFloat());
  }
}

void MeshDisplay::updateNormalsColor()
{
  bool showNormals = m_showNormals->getBool();

  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->updateNormals(showNormals, m_normalsColor->getOgreColor(), m_normalsAlpha->getFloat());
  }
}

void MeshDisplay::updateNormalsSize()
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->updateNormals(m_scalingFactor->getFloat());
  }
}

void MeshDisplay::updateVertexCosts()
{
  if (m_costUseCustomLimits->getBool())
  {
    if (m_costCache.count(m_selectVertexCostMap->getStdString()) != 0)
    {
      std::shared_ptr<MeshVisual> visual = getLatestVisual();
      if (visual)
      {
        visual->setVertexCosts(m_costCache[m_selectVertexCostMap->getStdString()], m_costColorType->getOptionInt(),
                               m_costLowerLimit->getFloat(), m_costUpperLimit->getFloat());
      }
    }
  }
  else
  {
    if (m_costCache.count(m_selectVertexCostMap->getStdString()) != 0)
    {
      std::shared_ptr<MeshVisual> visual = getLatestVisual();
      if (visual)
      {
        visual->setVertexCosts(m_costCache[m_selectVertexCostMap->getStdString()], m_costColorType->getOptionInt());
      }
    }
  }
  updateMesh();
}

void MeshDisplay::updateVertexColorsTopic()
{
  m_vertexColorsSubscriber.unsubscribe();
  delete m_colorsSynchronizer;

  m_vertexColorsSubscriber.subscribe(update_nh_, m_vertexColorsTopic->getTopicStd(), 1);
  m_colorsSynchronizer = new message_filters::Cache<mesh_msgs::MeshVertexColorsStamped>(m_vertexColorsSubscriber, 1);
  m_colorsSynchronizer->registerCallback(boost::bind(&MeshDisplay::incomingVertexColors, this, boost::placeholders::_1));
}

void MeshDisplay::updateVertexCostsTopic()
{
  m_vertexCostsSubscriber.unsubscribe();
  delete m_costsSynchronizer;

  m_vertexCostsSubscriber.subscribe(update_nh_, m_vertexCostsTopic->getTopicStd(), 4);
  m_costsSynchronizer = new message_filters::Cache<mesh_msgs::MeshVertexCostsStamped>(m_vertexCostsSubscriber, 1);
  m_costsSynchronizer->registerCallback(boost::bind(&MeshDisplay::incomingVertexCosts, this, boost::placeholders::_1));
}

void MeshDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  context_->queueRender();
}

void MeshDisplay::updateMaterialAndTextureServices()
{
  if (m_ignoreMsgs)
  {
    return;
  }

  // Check if the service names are valid
  std::string error;
  if (!ros::names::validate(m_materialServiceName->getStdString(), error) ||
      !ros::names::validate(m_textureServiceName->getStdString(), error))
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
    requestMaterials(m_lastUuid);
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

void MeshDisplay::updateVertexColorService()
{
  if (m_ignoreMsgs)
  {
    return;
  }

  // Check if the service name is valid
  std::string error;
  if (!ros::names::validate(m_vertexColorServiceName->getStdString(), error))
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
    requestVertexColors(m_lastUuid);
  }
  else
  {
    setStatus(rviz::StatusProperty::Warn, "Services", QString("The specified Vertex Color Service doesn't exist."));
  }
}

// =====================================================================================================================
// Data loading

void MeshDisplay::initialServiceCall()
{
  if (m_ignoreMsgs)
  {
    return;
  }

  ros::NodeHandle n;
  m_uuidClient = n.serviceClient<mesh_msgs::GetUUIDs>("get_uuid");

  mesh_msgs::GetUUIDs srv_uuids;
  if (m_uuidClient.call(srv_uuids))
  {
    std::vector<std::string> uuids = srv_uuids.response.uuids;

    if (uuids.size() > 0)
    {
      std::string uuid = uuids[0];

      ROS_INFO_STREAM("Initial data available for UUID=" << uuid);

      m_geometryClient = n.serviceClient<mesh_msgs::GetGeometry>("get_geometry");

      mesh_msgs::GetGeometry srv_geometry;
      srv_geometry.request.uuid = uuid;
      if (m_geometryClient.call(srv_geometry))
      {
        ROS_INFO_STREAM("Found geometry for UUID=" << uuid);
        mesh_msgs::MeshGeometryStamped::ConstPtr geometry =
            boost::make_shared<const mesh_msgs::MeshGeometryStamped>(srv_geometry.response.mesh_geometry_stamped);
        processMessage(geometry);
      }
      else
      {
        ROS_INFO_STREAM("Could not load geometry. Waiting for callback to trigger ... ");
      }
    }
  }
  else
  {
    ROS_INFO("No initial data available, waiting for callback to trigger ...");
  }
}

void MeshDisplay::processMessage(const mesh_msgs::MeshGeometryStamped::ConstPtr& meshMsg)
{
  if (m_ignoreMsgs)
  {
    return;
  }

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  if (!context_->getFrameManager()->getTransform(meshMsg->header.frame_id, meshMsg->header.stamp, position,
                                                 orientation))
  {
    ROS_ERROR("Error transforming from frame '%s' to frame '%s'", meshMsg->header.frame_id.c_str(),
              qPrintable(rviz::Display::fixed_frame_));
    return;
  }

  if (!m_lastUuid.empty() && meshMsg->uuid.compare(m_lastUuid) != 0)
  {
    ROS_WARN("Received geometry with new UUID!");
    m_costCache.clear();
    m_selectVertexCostMap->clearOptions();
    m_selectVertexCostMap->addOption("-- None --", 0);
  }

  m_lastUuid = meshMsg->uuid;

  // set Geometry
  std::shared_ptr<Geometry> mesh(std::make_shared<Geometry>());
  for (const geometry_msgs::Point& v : meshMsg->mesh_geometry.vertices)
  {
    Vertex vertex;
    vertex.x = v.x;
    vertex.y = v.y;
    vertex.z = v.z;
    mesh->vertices.push_back(vertex);
  }
  for (const mesh_msgs::MeshTriangleIndices& f : meshMsg->mesh_geometry.faces)
  {
    Face face;
    face.vertexIndices[0] = f.vertex_indices[0];
    face.vertexIndices[1] = f.vertex_indices[1];
    face.vertexIndices[2] = f.vertex_indices[2];
    mesh->faces.push_back(face);
  }
  setGeometry(mesh);
  setPose(position, orientation);

  // set Normals
  std::vector<Normal> normals;
  for (const geometry_msgs::Point& n : meshMsg->mesh_geometry.vertex_normals)
  {
    Normal normal(n.x, n.y, n.z);
    normals.push_back(normal);
  }
  setVertexNormals(normals);

  requestVertexColors(meshMsg->uuid);
  requestMaterials(meshMsg->uuid);
}

void MeshDisplay::incomingGeometry(const mesh_msgs::MeshGeometryStamped::ConstPtr& meshMsg)
{
  m_messagesReceived++;
  setStatus(rviz::StatusProperty::Ok, "Topic", QString::number(m_messagesReceived) + " messages received");
  processMessage(meshMsg);
}

void MeshDisplay::incomingVertexColors(const mesh_msgs::MeshVertexColorsStamped::ConstPtr& colorsStamped)
{
  if (colorsStamped->uuid.compare(m_lastUuid) != 0)
  {
    ROS_ERROR("Received vertex colors, but UUIDs dont match!");
    return;
  }

  std::vector<Color> vertexColors;
  for (const std_msgs::ColorRGBA c : colorsStamped->mesh_vertex_colors.vertex_colors)
  {
    Color color(c.r, c.g, c.b, c.a);
    vertexColors.push_back(color);
  }

  setVertexColors(vertexColors);
}

void MeshDisplay::incomingVertexCosts(const mesh_msgs::MeshVertexCostsStamped::ConstPtr& costsStamped)
{
  if (costsStamped->uuid.compare(m_lastUuid) != 0)
  {
    ROS_ERROR("Received vertex costs, but UUIDs dont match!");
    return;
  }

  cacheVertexCosts(costsStamped->type, costsStamped->mesh_vertex_costs.costs);
  updateVertexCosts();
}

void MeshDisplay::requestVertexColors(std::string uuid)
{
  if (m_ignoreMsgs)
  {
    return;
  }

  mesh_msgs::GetVertexColors srv;
  srv.request.uuid = uuid;
  if (m_vertexColorClient.call(srv))
  {
    ROS_INFO("Successful vertex colors service call!");
    mesh_msgs::MeshVertexColorsStamped::ConstPtr meshVertexColors =
        boost::make_shared<const mesh_msgs::MeshVertexColorsStamped>(srv.response.mesh_vertex_colors_stamped);

    std::vector<Color> vertexColors;
    for (const std_msgs::ColorRGBA c : meshVertexColors->mesh_vertex_colors.vertex_colors)
    {
      Color color(c.r, c.g, c.b, c.a);
      vertexColors.push_back(color);
    }

    setVertexColors(vertexColors);
  }
  else
  {
    ROS_INFO("Failed vertex colors service call!");
  }
}

void MeshDisplay::requestMaterials(std::string uuid)
{
  if (m_ignoreMsgs)
  {
    return;
  }

  mesh_msgs::GetMaterials srv;
  srv.request.uuid = uuid;
  if (m_materialsClient.call(srv))
  {
    ROS_INFO("Successful materials service call!");

    mesh_msgs::MeshMaterialsStamped::ConstPtr meshMaterialsStamped =
        boost::make_shared<const mesh_msgs::MeshMaterialsStamped>(srv.response.mesh_materials_stamped);

    std::vector<Material> materials(meshMaterialsStamped->mesh_materials.materials.size());
    for (int i = 0; i < meshMaterialsStamped->mesh_materials.materials.size(); i++)
    {
      const mesh_msgs::MeshMaterial& mat = meshMaterialsStamped->mesh_materials.materials[i];
      materials[i].textureIndex = mat.texture_index;
      materials[i].color = Color(mat.color.r, mat.color.g, mat.color.b, mat.color.a);
    }
    for (int i = 0; i < meshMaterialsStamped->mesh_materials.clusters.size(); i++)
    {
      const mesh_msgs::MeshFaceCluster& clu = meshMaterialsStamped->mesh_materials.clusters[i];

      uint32_t materialIndex = meshMaterialsStamped->mesh_materials.cluster_materials[i];
      const mesh_msgs::MeshMaterial& mat = meshMaterialsStamped->mesh_materials.materials[materialIndex];

      for (uint32_t face_index : clu.face_indices)
      {
        materials[i].faceIndices.push_back(face_index);
      }
    }

    std::vector<TexCoords> textureCoords;
    for (const mesh_msgs::MeshVertexTexCoords& coord : meshMaterialsStamped->mesh_materials.vertex_tex_coords)
    {
      textureCoords.push_back(TexCoords(coord.u, coord.v));
    }

    setMaterials(materials, textureCoords);

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
          mesh_msgs::MeshTexture::ConstPtr textureMsg =
              boost::make_shared<const mesh_msgs::MeshTexture>(texSrv.response.texture);

          Texture texture;
          texture.width = textureMsg->image.width;
          texture.height = textureMsg->image.height;
          texture.channels = textureMsg->image.step;
          texture.pixelFormat = textureMsg->image.encoding;
          texture.data = textureMsg->image.data;

          addTexture(texture, textureMsg->texture_index);
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

void MeshDisplay::cacheVertexCosts(std::string layer, const std::vector<float>& costs)
{
  ROS_INFO_STREAM("Cache vertex cost map '" << layer << "' for UUID ");

  // insert into cache
  auto it = m_costCache.find(layer);
  if (it != m_costCache.end())
  {
    ROS_INFO_STREAM("The cost layer \"" << layer << "\" has been updated.");
    m_costCache.erase(layer);
  }
  else
  {
    ROS_INFO_STREAM("The cost layer \"" << layer << "\" has been added.");
    m_selectVertexCostMap->addOptionStd(layer, m_selectVertexCostMap->numChildren());
  }

  m_costCache.insert(std::pair<std::string, std::vector<float>>(layer, costs));
}

std::shared_ptr<MeshVisual> MeshDisplay::getLatestVisual()
{
  if (m_visuals.empty())
  {
    return nullptr;
  }
  return m_visuals.back();
}

std::shared_ptr<MeshVisual> MeshDisplay::addNewVisual()
{
  int randomId = (int)((double)rand() / RAND_MAX * 9998);
  m_visuals.push(std::make_shared<MeshVisual>(context_, 0, 0, randomId));

  int bufferCapacity = m_bufferSize->getInt();
  if (m_ignoreMsgs)
  {
    bufferCapacity = 1;
  }

  if (m_visuals.size() > bufferCapacity)
  {
    m_visuals.pop();
  }

  return m_visuals.back();
}

}  // End namespace rviz_map_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_map_plugin::MeshDisplay, rviz::Display)
