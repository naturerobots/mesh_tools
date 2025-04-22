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

#include <rviz_mesh_tools_plugins/MeshDisplay.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>


#include "mesh_msgs/srv/get_vertex_colors.hpp"
#include "mesh_msgs/srv/get_materials.hpp"
#include "mesh_msgs/srv/get_geometry.hpp"
#include "mesh_msgs/srv/get_texture.hpp"
#include "mesh_msgs/srv/get_uui_ds.hpp"

#include "mesh_msgs/msg/mesh_vertex_colors_stamped.hpp"
#include "mesh_msgs/msg/mesh_geometry_stamped.hpp"


#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <rviz_common/display.hpp>

#include "rclcpp/executor.hpp"
#include "rmw/validate_full_topic_name.h"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace {
  constexpr struct {
    const int fixed_color = 0;
    const int vertex_color = 1;
    const int textures = 2;
    const int vertex_costs = 3;
    const int hide_faces = 4;
  } display_type_option;
}

namespace rviz_mesh_tools_plugins
{
MeshDisplay::MeshDisplay() 
: rviz_common::Display()
, m_ignoreMsgs(false)
, m_meshQos(rclcpp::SystemDefaultsQoS())
, m_vertexColorsQos(rclcpp::SystemDefaultsQoS())
, m_vertexCostsQos(rclcpp::SystemDefaultsQoS())
{
  // mesh topic
  m_meshTopic = new rviz_common::properties::RosTopicProperty(
      "Geometry Topic", "", QString::fromStdString(rosidl_generator_traits::name<mesh_msgs::msg::MeshGeometryStamped>()),
      "Geometry topic to subscribe to.", this, SLOT(updateMeshGeometrySubscription()), this);
      
  m_meshTopicQos = new rviz_common::properties::QosProfileProperty(m_meshTopic, m_meshQos);

  // buffer size / amount of meshes visualized
  m_bufferSize = new rviz_common::properties::IntProperty("Buffer Size", 1, "Number of meshes visualized", this, SLOT(updateBufferSize()));
  m_bufferSize->setMin(1);

  // Display Type
  {
    m_displayType = new rviz_common::properties::EnumProperty("Display Type", "Fixed Color", "Select Display Type for Mesh", this,
                                           SLOT(updateDisplayType()), this);
    m_displayType->addOption("Fixed Color", display_type_option.fixed_color);
    m_displayType->addOption("Vertex Color", display_type_option.vertex_color);
    m_displayType->addOption("Textures", display_type_option.textures);
    m_displayType->addOption("Vertex Costs", display_type_option.vertex_costs);
    m_displayType->addOption("Hide Faces", display_type_option.hide_faces);

    // Fixed Color
    {
      // face color properties
      m_facesColor = new rviz_common::properties::ColorProperty("Faces Color", QColor(0, 255, 0), "The color of the faces.", m_displayType,
                                             SLOT(updateMeshMaterial()), this);

      // face alpha properties
      m_facesAlpha = new rviz_common::properties::FloatProperty("Faces Alpha", 1.0, "The alpha-value of the faces", m_displayType,
                                             SLOT(updateMeshMaterial()), this);
      m_facesAlpha->setMin(0);
      m_facesAlpha->setMax(1);
    }

    // Vertex Color
    {
      m_vertexColorsTopic = new rviz_common::properties::RosTopicProperty(
          "Vertex Colors Topic", "",
          QString::fromStdString(rosidl_generator_traits::name<mesh_msgs::msg::MeshVertexColorsStamped>()),
          "Vertex color topic to subscribe to.", m_displayType, SLOT(updateVertexColorsSubscription()), this);

      m_vertexColorsTopicQos = new rviz_common::properties::QosProfileProperty(m_vertexColorsTopic, m_vertexColorsQos);

      m_vertexColorServiceName = new rviz_common::properties::StringProperty("Vertex Color Service Name", "get_vertex_colors",
                                                          "Name of the Vertex Color Service to request Vertex Colors "
                                                          "from.",
                                                          m_displayType, SLOT(updateVertexColorService()), this);
    }

    // Textures
    {
      m_showTexturedFacesOnly = new rviz_common::properties::BoolProperty("Show textured faces only", false, "Show textured faces only",
                                                       m_displayType, SLOT(updateMeshMaterial()), this);

      m_materialServiceName = new rviz_common::properties::StringProperty("Material Service Name", "get_materials",
                                                       "Name of the Matrial Service to request Materials from.",
                                                       m_displayType, SLOT(updateMaterialAndTextureServices()), this);

      m_textureServiceName = new rviz_common::properties::StringProperty("Texture Service Name", "get_texture",
                                                      "Name of the Texture Service to request Textures from.",
                                                      m_displayType, SLOT(updateMaterialAndTextureServices()), this);
    }

    // Vertex Costs
    {
      m_costColorType = new rviz_common::properties::EnumProperty("Color Scale", "Rainbow",
                                               "Select color scale for vertex costs. Mesh will update when new data "
                                               "arrives.",
                                               m_displayType, SLOT(updateVertexCosts()), this);
      m_costColorType->addOption("Rainbow", 0);
      m_costColorType->addOption("Red Green", 1);

      

      m_vertexCostsTopic = new rviz_common::properties::RosTopicProperty(
          "Vertex Costs Topic", "",
          QString::fromStdString(rosidl_generator_traits::name<mesh_msgs::msg::MeshVertexCostsStamped>()),
          "Vertex cost topic to subscribe to.", m_displayType, SLOT(updateVertexCostsSubscription()), this);

      m_vertexCostsTopicQos = new rviz_common::properties::QosProfileProperty(m_vertexCostsTopic, m_vertexCostsQos);

      m_selectVertexCostMap = new rviz_common::properties::EnumProperty("Vertex Costs Type", "-- None --",
                                                     "Select the type of vertex cost map to be displayed. New types "
                                                     "will appear here when a new message arrives.",
                                                     m_displayType, SLOT(updateVertexCosts()), this);
      m_selectVertexCostMap->addOption("-- None --", 0);

      m_costUseCustomLimits = new rviz_common::properties::BoolProperty("Use Custom limits", false, "Use custom vertex cost limits",
                                                     m_displayType, SLOT(updateVertexCosts()), this);

      // custom cost limits
      {
        m_costLowerLimit = new rviz_common::properties::FloatProperty("Vertex Costs Lower Limit", 0.0, "Vertex costs lower limit",
                                                   m_costUseCustomLimits, SLOT(updateVertexCosts()), this);
        m_costUpperLimit = new rviz_common::properties::FloatProperty("Vertex Costs Upper Limit", 1.0, "Vertex costs upper limit",
                                                   m_costUseCustomLimits, SLOT(updateVertexCosts()), this);
      }
    }
    updateDisplayType(); // ensures that DisplayType children are correctly hidden / visible for the chosen default option
  }

  // Wireframe
  {
    m_showWireframe =
        new rviz_common::properties::BoolProperty("Show Wireframe", true, "Show Wireframe", this, SLOT(updateWireframe()), this);

    // wireframe color property
    m_wireframeColor = new rviz_common::properties::ColorProperty("Wireframe Color", QColor(0, 0, 0), "The color of the wireframe.",
                                               m_showWireframe, SLOT(updateWireframe()), this);
    // wireframe alpha property
    m_wireframeAlpha = new rviz_common::properties::FloatProperty("Wireframe Alpha", 1.0, "The alpha-value of the wireframe",
                                               m_showWireframe, SLOT(updateWireframe()), this);
    m_wireframeAlpha->setMin(0);
    m_wireframeAlpha->setMax(1);
  }

  // Normals
  {
    m_showNormals = new rviz_common::properties::BoolProperty("Show Normals", true, "Show Normals", this, SLOT(updateNormals()), this);

    m_normalsColor = new rviz_common::properties::ColorProperty("Normals Color", QColor(255, 0, 255), "The color of the normals.",
                                             m_showNormals, SLOT(updateNormalsColor()), this);
    m_normalsAlpha = new rviz_common::properties::FloatProperty("Normals Alpha", 1.0, "The alpha-value of the normals", m_showNormals,
                                             SLOT(updateNormalsColor()), this);
    m_normalsAlpha->setMin(0);
    m_normalsAlpha->setMax(1);
    m_scalingFactor = new rviz_common::properties::FloatProperty("Normals Scaling Factor", 0.1, "Scaling factor of the normals",
                                              m_showNormals, SLOT(updateNormalsSize()), this);
  }

  // Culling
  {
    m_cullingMode = new rviz_common::properties::EnumProperty("Culling Mode (HW)", "None", "Select Hardware Culling Mode for Mesh", this,
      SLOT(updateCullingMode()), this);
    m_cullingMode->addOption("None", Ogre::CULL_NONE);
    m_cullingMode->addOption("Clockwise", Ogre::CULL_CLOCKWISE);
    m_cullingMode->addOption("Anticlockwise", Ogre::CULL_ANTICLOCKWISE);
  }
}

MeshDisplay::~MeshDisplay()
{
  unsubscribe();
}

// void MeshDisplay::load(const rviz_common::Config& config)
// {
//   RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), ": MESH LOAD CONFIG...");

//   rviz_common::Display::load(config);
// }

void MeshDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();

  connect(
    reinterpret_cast<QObject *>(context_->getTransformationManager()),
    SIGNAL(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)),
    this,
    SLOT(transformerChangedCallback()));

  m_meshTopic->initialize(context_->getRosNodeAbstraction());
  m_vertexColorsTopic->initialize(context_->getRosNodeAbstraction());
  m_vertexCostsTopic->initialize(context_->getRosNodeAbstraction());

  m_meshTopicQos->initialize(
      [this](rclcpp::QoS profile) {
        m_meshQos = profile;
        updateMeshGeometrySubscription();
      });

  m_vertexColorsTopicQos->initialize(
      [this](rclcpp::QoS profile) {
        m_vertexColorsQos = profile;
        updateVertexColorsSubscription();
      });

  m_vertexCostsTopicQos->initialize(
      [this](rclcpp::QoS profile) {
        m_vertexCostsQos = profile;
        updateVertexCostsSubscription();
      });

  // Initialize service clients
  //m_vertexColorClient = node->create_client<mesh_msgs::srv::GetVertexColors>(m_vertexColorServiceName->getStdString());
  //m_materialsClient = node->create_client<mesh_msgs::srv::GetMaterials>(m_materialServiceName->getStdString());
  //m_textureClient = node->create_client<mesh_msgs::srv::GetTexture>(m_textureServiceName->getStdString());
  //m_uuidClient = node->create_client<mesh_msgs::srv::GetUUIDs>("get_uuid");
  //m_geometryClient = node->create_client<mesh_msgs::srv::GetGeometry>("get_geometry");
}

void MeshDisplay::update(float wall_dt, float ros_dt) 
{
  (void) wall_dt;
  (void) ros_dt;

  this->transformMesh();
}

void MeshDisplay::transformMesh()
{
  auto logger = rclcpp::get_logger("rviz_mesh_tools_plugins");
  RCLCPP_DEBUG(logger, "MeshDisplay::transformMesh() - called");

  // TODO: Check if we already received a Mesh. Using getLatestVisual segfaults.

  // Happens if this is called after the constructor before onInitialize()
  if (!context_)
  {
    RCLCPP_DEBUG(logger, "MeshDisplay::transformMesh() - context_ is not initialized");
    return;
  }

  // Update the pose relative to the fixed frame (Ogres top level scene node)
  if (!scene_node_)
  {
    RCLCPP_DEBUG(logger, "MeshDisplay::transformMesh() - scene_node_ is not initialized");
    return;
  }
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(m_meshFrame, position, orientation))
  {
    this->setMissingTransformToFixedFrame(m_meshFrame);
    RCLCPP_DEBUG(logger, "MeshDisplay::transformMesh() - missing transform");
    return;
  }

  this->setTransformOk();

  this->setPose(position, orientation);
  RCLCPP_DEBUG(
    logger, "MeshDisplay::transformMesh() - Set pose to [%f, %f, %f] [%f, %f, %f, %f]",
    position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w
  );
}

void MeshDisplay::onEnable()
{
  rviz_common::Display::onEnable();
  m_messagesReceived = 0;
  
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if(visual)
  {
    visual->show();
  }

  updateAllSubscriptions();
}

void MeshDisplay::onDisable()
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if(visual)
  {
    visual->hide();
  }

  unsubscribe();
  if (!m_ignoreMsgs)
  {
    reset();
  }
}

void MeshDisplay::transformerChangedCallback()
{
  updateMeshGeometrySubscription();
}

void MeshDisplay::fixedFrameChanged()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "MeshDisplay::fixedFrameChanged() - called"
  );

  if (m_tfMeshFilter) 
  {
    m_tfMeshFilter->setTargetFrame(fixed_frame_.toStdString());
  }

  this->transformMesh();
}

void MeshDisplay::reset()
{
  Display::reset();
  if (m_tfMeshFilter) {
    m_tfMeshFilter->clear();
  }
  std::queue<std::shared_ptr<MeshVisual>>().swap(m_visuals); // clear visuals
}

void MeshDisplay::updateMeshGeometrySubscription()
{
  if (m_meshTopic->isEmpty()) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString("Error subscribing: Empty mesh topic name"));
    return;
  }
  try
  {
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

    m_meshSubscriber.subscribe(node, m_meshTopic->getTopicStd(), m_meshQos.get_rmw_qos_profile());

    m_tfMeshFilter = std::make_shared<tf2_ros::RVizMessageFilter<mesh_msgs::msg::MeshGeometryStamped> >(m_meshSubscriber,
        *context_->getFrameManager()->getTransformer(), fixed_frame_.toStdString(), 2,
        node);
    m_tfMeshFilter->connectInput(m_meshSubscriber);
    m_tfMeshFilter->registerCallback(std::bind(&MeshDisplay::meshGeometryCallback, this, _1));

    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  }
  catch (std::runtime_error& e)
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
}

void MeshDisplay::updateAllSubscriptions()
{
  if (!isEnabled() || m_ignoreMsgs)
  {
    return;
  }

  updateMeshGeometrySubscription();
  updateVertexColorsSubscription();
  updateVertexCostsSubscription();

  // TODO
  // initialServiceCall();
}

void MeshDisplay::unsubscribe()
{
  m_meshSubscriber.unsubscribe();
  m_vertexColorsSubscriber.unsubscribe();
  m_vertexCostsSubscriber.unsubscribe();

  m_tfMeshFilter.reset();
  m_colorsMsgCache.reset();
  m_costsMsgCache.reset();
}

void MeshDisplay::ignoreIncomingMessages()
{
  m_ignoreMsgs = true;
  unsubscribe();
  updateMeshMaterial();

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
    updateMeshMaterial();
    updateNormals();
    updateWireframe();
    updateCullingMode();
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "Display", "");
}

void MeshDisplay::setMapFrame(const std::string& frame)
{
  this->m_meshFrame = frame;
}

void MeshDisplay::setVertexColors(vector<Color>& vertexColors)
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->setVertexColors(vertexColors);
  }
  updateMeshMaterial();
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
  updateMeshMaterial();
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

void MeshDisplay::updateDisplayType()
{
  RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Update display type"); 
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
  switch (m_displayType->getOptionInt())
  {
    default:
    case display_type_option.fixed_color:
      m_facesColor->show();
      m_facesAlpha->show();
      break;
    case display_type_option.vertex_color:
      if (!m_ignoreMsgs)
      {
        m_vertexColorsTopic->show();
        m_vertexColorServiceName->show();
        if (!m_colorsMsgCache) // checks whether we are not subscribed yet, avoids resetting the subscription when something else changes in the display type
        {
          updateVertexColorsSubscription();
        }
      }
      break;
    case display_type_option.textures:
      m_showTexturedFacesOnly->show();
      if (!m_ignoreMsgs)
      {
        m_materialServiceName->show();
        m_textureServiceName->show();
      }
      break;
    case display_type_option.vertex_costs:
      m_costColorType->show();
      if (!m_ignoreMsgs)
      {
        m_vertexCostsTopic->show();
        if (!m_costsMsgCache) // checks whether we are not subscribed yet, avoids resetting the subscription when something else changes in the display type
        {
          updateVertexCostsSubscription();
        }
      }
      m_selectVertexCostMap->show();
      m_costUseCustomLimits->show();
      if (m_costUseCustomLimits->getBool())
      {
        m_costLowerLimit->show();
        m_costUpperLimit->show();
      }
      break;
  }

  updateMeshMaterial();
}

void MeshDisplay::updateMeshMaterial()
{
  bool showFaces = false;
  bool showTextures = false;
  bool showVertexColors = false;
  bool showVertexCosts = false;
  switch (m_displayType->getOptionInt())
  {
    default:
    case display_type_option.fixed_color:
      showFaces = true;
      break;
    case display_type_option.vertex_color:
      showFaces = true;
      showVertexColors = true;
      break;
    case display_type_option.textures:
      showFaces = true;
      showTextures = true;
      break;
    case display_type_option.vertex_costs:
      showFaces = true;
      showVertexCosts = true;
      break;
    case display_type_option.hide_faces:
      break;
  }

  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Updating visual's mesh material");
    visual->updateMaterial(showFaces, m_facesColor->getOgreColor(), m_facesAlpha->getFloat(), showVertexColors,
                           showVertexCosts, showTextures, m_showTexturedFacesOnly->getBool());
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
  updateMeshMaterial();
}

void MeshDisplay::updateVertexColorsSubscription()
{
  if (!m_vertexColorsTopic->getHidden())
  {
    if (!m_vertexColorsTopic->getTopicStd().empty()) 
    {
      auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
      m_vertexColorsSubscriber.subscribe(node, m_vertexColorsTopic->getTopicStd(), m_vertexColorsQos.get_rmw_qos_profile());

      m_colorsMsgCache = std::make_shared<message_filters::Cache<mesh_msgs::msg::MeshVertexColorsStamped>>(m_vertexColorsSubscriber, 1);
      m_colorsMsgCache->registerCallback(std::bind(&MeshDisplay::vertexColorsCallback, this, _1));
    } else {
      setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", QString("Error subscribing: Empty color topic name"));
    }
  }
}

void MeshDisplay::updateVertexCostsSubscription()
{
  if (!m_vertexCostsTopic->getHidden())
  {
    if (!m_vertexCostsTopic->getTopicStd().empty()) 
    {
      auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
      m_vertexCostsSubscriber.subscribe(node, m_vertexCostsTopic->getTopicStd(), m_vertexCostsQos.get_rmw_qos_profile());

      m_costsMsgCache = std::make_shared<message_filters::Cache<mesh_msgs::msg::MeshVertexCostsStamped>>(m_vertexCostsSubscriber, 1);
      m_costsMsgCache->registerCallback(std::bind(&MeshDisplay::vertexCostsCallback, this, _1));
    } else {
      setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", QString("Error subscribing: Empty cost topic name"));
    }
  }
}

void MeshDisplay::updateMaterialAndTextureServices()
{
  if (m_ignoreMsgs)
  {
    return;
  }

  // Check if the service names are valid
  if (rmw_validate_full_topic_name(m_materialServiceName->getStdString().c_str(), nullptr, nullptr) != RMW_TOPIC_VALID ||
      rmw_validate_full_topic_name(m_textureServiceName->getStdString().c_str(), nullptr, nullptr) != RMW_TOPIC_VALID )
  {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Services", QString("The service name contains an invalid character."));
    return;
  }

  // Update material and texture service clients
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  m_materialsClient = node->create_client<mesh_msgs::srv::GetMaterials>(m_materialServiceName->getStdString());
  m_textureClient = node->create_client<mesh_msgs::srv::GetTexture>(m_textureServiceName->getStdString());

  if(m_materialsClient->wait_for_service(std::chrono::seconds(5)))
  {
    requestMaterials(m_lastUuid);

    if (m_textureClient->wait_for_service(1s))
    {
      setStatus(rviz_common::properties::StatusProperty::Ok, "Services", "Material and Texture Service OK");
    }
    else
    {
      setStatus(rviz_common::properties::StatusProperty::Warn, "Services", QString("The specified Texture Service doesn't exist."));
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Unable to mesh_msgs::srv::GetMaterials service. Start vision_node first.");
    setStatus(rviz_common::properties::StatusProperty::Warn, "Services", QString("The specified Material Service doesn't exist."));
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
  
  if (rmw_validate_full_topic_name(m_vertexColorServiceName->getStdString().c_str(), nullptr, nullptr) != RMW_TOPIC_VALID)
  {    
    setStatus(rviz_common::properties::StatusProperty::Warn, "Services", QString("The service name contains an invalid character."));
    return;
  }

  // Update vertex color service client
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  m_vertexColorClient = node->create_client<mesh_msgs::srv::GetVertexColors>(m_vertexColorServiceName->getStdString());
  if(m_vertexColorClient->wait_for_service(std::chrono::seconds(5)))
  {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Services", "Vertex Color Service OK");
    requestVertexColors(m_lastUuid);
  }
  else
  {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Services", QString("The specified Vertex Color Service doesn't exist."));
  }
}

void MeshDisplay::updateCullingMode()
{
  std::shared_ptr<MeshVisual> visual = getLatestVisual();
  if (visual)
  {
    visual->setCullingMode(static_cast<Ogre::CullingMode>(m_cullingMode->getOptionInt()));
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

  auto req_uuids = std::make_shared<mesh_msgs::srv::GetUUIDs::Request>();
  auto fut_uuids = m_uuidClient->async_send_request(req_uuids);

  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  if(rclcpp::spin_until_future_complete(node, fut_uuids) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res_uuids = fut_uuids.get();

    std::vector<std::string> uuids = res_uuids->uuids;

    if(uuids.size() > 0)
    {
      std::string uuid = uuids[0];

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Initial data available for UUID=" << uuid);
      
      // mesh_msgs::srv::GetGeometry srv_geometry;
      auto req_geometry = std::make_shared<mesh_msgs::srv::GetGeometry::Request>();
      req_geometry->uuid = uuid;

      auto fut_geometry = m_geometryClient->async_send_request(req_geometry);
      auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
      if(rclcpp::spin_until_future_complete(node, fut_geometry) == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto res_geometry = fut_geometry.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found geometry for UUID=" << uuid);
        processMessage(res_geometry->mesh_geometry_stamped);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Failed to receive mesh_msgs::srv::GetGeometry service response");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Could not load geometry. Waiting for callback to trigger ... "); 
      }
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "No initial data available, waiting for callback to trigger ...");
  }
}

void MeshDisplay::processMessage(
  const mesh_msgs::msg::MeshGeometryStamped& meshMsg)
{
  if (m_ignoreMsgs)
  {
    return;
  }

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  if (!context_->getFrameManager()->getTransform(meshMsg.header.frame_id, meshMsg.header.stamp, position,
                                                 orientation))
  {
    RCLCPP_ERROR(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Error transforming from frame '%s' to frame '%s'", meshMsg.header.frame_id.c_str(),
              qPrintable(rviz_common::Display::fixed_frame_));
    return;
  }

  if (!m_lastUuid.empty() && meshMsg.uuid.compare(m_lastUuid) != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Received geometry with new UUID!");
    m_costCache.clear();
    m_selectVertexCostMap->clearOptions();
    m_selectVertexCostMap->addOption("-- None --", 0);
  }

  m_lastUuid = meshMsg.uuid;

  // Reuse memory if already existing

  // set Geometry
  std::shared_ptr<Geometry> mesh(std::make_shared<Geometry>());
  for (const geometry_msgs::msg::Point& v : meshMsg.mesh_geometry.vertices)
  {
    Vertex vertex;
    vertex.x = v.x;
    vertex.y = v.y;
    vertex.z = v.z;
    mesh->vertices.push_back(vertex);
  }
  for (const mesh_msgs::msg::MeshTriangleIndices& f : meshMsg.mesh_geometry.faces)
  {
    Face face;
    face.vertexIndices[0] = f.vertex_indices[0];
    face.vertexIndices[1] = f.vertex_indices[1];
    face.vertexIndices[2] = f.vertex_indices[2];
    mesh->faces.push_back(face);
  }
  setGeometry(mesh);
  setPose(position, orientation);
  m_meshFrame = meshMsg.header.frame_id;

  // set Normals
  std::vector<Normal> normals;
  for (const geometry_msgs::msg::Point& n : meshMsg.mesh_geometry.vertex_normals)
  {
    Normal normal(n.x, n.y, n.z);
    normals.push_back(normal);
  }
  setVertexNormals(normals);

  // TODO service stuff
  // std::cout << "requestVertexColors" << std::endl;
  // requestVertexColors(meshMsg.uuid);
  // std::cout << "requestMaterials" << std::endl;
  // requestMaterials(meshMsg.uuid);
  // std::cout << "done." << std::endl;
}

void MeshDisplay::meshGeometryCallback(
  const mesh_msgs::msg::MeshGeometryStamped::ConstSharedPtr& meshMsg)
{
  m_messagesReceived++;
  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", QString::number(m_messagesReceived) + " messages received");
  processMessage(*meshMsg);

  // check for cached color and cost msgs that might have arrived earlier:
  if (m_colorsMsgCache) {
    const auto colorMsgs = m_colorsMsgCache->getInterval(meshMsg->header.stamp, meshMsg->header.stamp); // get msgs where header.stamp match exactly (this should usually be only one msg)
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Got " << colorMsgs.size() << " color msgs from cache with header.stamp=" << rclcpp::Time(meshMsg->header.stamp).seconds());
    for (const mesh_msgs::msg::MeshVertexColorsStamped::ConstSharedPtr& colorMsg : colorMsgs) {
      if (colorMsg->uuid == meshMsg->uuid) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found cached color msg with matching UUID, applying its information now");
        vertexColorsCallback(colorMsg);
      } else {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found cached color msg, but UUIDs do not match. Ignoring color msg.");
      }
    }
  }
  if (m_costsMsgCache) {
    const auto costMsgs = m_costsMsgCache->getInterval(meshMsg->header.stamp, meshMsg->header.stamp); // get msgs where header.stamp match exactly (this should usually be only one msg)
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Got " << costMsgs.size() << " cost msgs from cache with header.stamp=" << rclcpp::Time(meshMsg->header.stamp).seconds());
    for (const mesh_msgs::msg::MeshVertexCostsStamped::ConstSharedPtr& costMsg : costMsgs) {
      if (costMsg->uuid == meshMsg->uuid) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found cached cost msg with matching UUID, applying its information now");
        vertexCostsCallback(costMsg);
      } else {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found cached cost msg, but UUIDs do not match. Ignoring cost msg.");
      }
    }
  }
}

void MeshDisplay::vertexColorsCallback(
  const mesh_msgs::msg::MeshVertexColorsStamped::ConstSharedPtr& colorsStamped)
{
  if (colorsStamped->uuid.compare(m_lastUuid) != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Received vertex colors, but its UUID does not match the latest mesh geometry UUID. Not updating colors.");
    return;
  }

  std::vector<Color> vertexColors;
  for (const std_msgs::msg::ColorRGBA c : colorsStamped->mesh_vertex_colors.vertex_colors)
  {
    Color color(c.r, c.g, c.b, c.a);
    vertexColors.push_back(color);
  }

  setVertexColors(vertexColors);
}

void MeshDisplay::vertexCostsCallback(
  const mesh_msgs::msg::MeshVertexCostsStamped::ConstSharedPtr& costsStamped)
{
  if (costsStamped->uuid.compare(m_lastUuid) != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Received vertex costs, but its UUID does not match the latest mesh geometry UUID. Not updating costs.");
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

  auto req_vertex_colors = std::make_shared<mesh_msgs::srv::GetVertexColors::Request>();
  req_vertex_colors->uuid = uuid;

  auto fut_vertex_colors = m_vertexColorClient->async_send_request(req_vertex_colors);

  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  if(rclcpp::spin_until_future_complete(node, fut_vertex_colors) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res_vertex_colors = fut_vertex_colors.get();
    RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Successful vertex colors service call!");

    std::vector<Color> vertexColors;
    for (const std_msgs::msg::ColorRGBA c : res_vertex_colors->mesh_vertex_colors_stamped.mesh_vertex_colors.vertex_colors)
    {
      Color color(c.r, c.g, c.b, c.a);
      vertexColors.push_back(color);
    }

    setVertexColors(vertexColors);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Failed vertex colors service call!");
  }
}

void MeshDisplay::requestMaterials(std::string uuid)
{
  if (m_ignoreMsgs)
  {
    return;
  }

  mesh_msgs::srv::GetMaterials srv;
  auto req_materials = std::make_shared<mesh_msgs::srv::GetMaterials::Request>();
  req_materials->uuid = uuid;

  auto fut_materials = m_materialsClient->async_send_request(req_materials);
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // TODO: this doesnt work
  if(rclcpp::spin_until_future_complete(node, fut_materials) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res_materials = fut_materials.get();

    RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Successful materials service call!");

    const mesh_msgs::msg::MeshMaterialsStamped& meshMaterialsStamped =
        res_materials->mesh_materials_stamped;

    std::vector<Material> materials(meshMaterialsStamped.mesh_materials.materials.size());
    for (int i = 0; i < meshMaterialsStamped.mesh_materials.materials.size(); i++)
    {
      const mesh_msgs::msg::MeshMaterial& mat = meshMaterialsStamped.mesh_materials.materials[i];
      materials[i].textureIndex = mat.texture_index;
      materials[i].color = Color(mat.color.r, mat.color.g, mat.color.b, mat.color.a);
    }
    for (int i = 0; i < meshMaterialsStamped.mesh_materials.clusters.size(); i++)
    {
      const mesh_msgs::msg::MeshFaceCluster& clu = meshMaterialsStamped.mesh_materials.clusters[i];

      uint32_t materialIndex = meshMaterialsStamped.mesh_materials.cluster_materials[i];
      const mesh_msgs::msg::MeshMaterial& mat = meshMaterialsStamped.mesh_materials.materials[materialIndex];

      for (uint32_t face_index : clu.face_indices)
      {
        materials[i].faceIndices.push_back(face_index);
      }
    }

    std::vector<TexCoords> textureCoords;
    for (const mesh_msgs::msg::MeshVertexTexCoords& coord : meshMaterialsStamped.mesh_materials.vertex_tex_coords)
    {
      textureCoords.push_back(TexCoords(coord.u, coord.v));
    }

    setMaterials(materials, textureCoords);

    for (mesh_msgs::msg::MeshMaterial material : meshMaterialsStamped.mesh_materials.materials)
    {
      if (material.has_texture)
      {
        auto req_texture = std::make_shared<mesh_msgs::srv::GetTexture::Request>();

        req_texture->uuid = uuid;
        req_texture->texture_index = material.texture_index;
        
        auto fut_texture = m_textureClient->async_send_request(req_texture);

        auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        if(rclcpp::spin_until_future_complete(node, fut_texture) == rclcpp::FutureReturnCode::SUCCESS)
        {
          auto res_texture = fut_texture.get();
          RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Successful texture service call with index %d!", material.texture_index);
          const mesh_msgs::msg::MeshTexture& textureMsg =
              res_texture->texture;

          Texture texture;
          texture.width = textureMsg.image.width;
          texture.height = textureMsg.image.height;
          texture.channels = textureMsg.image.step;
          texture.pixelFormat = textureMsg.image.encoding;
          texture.data = textureMsg.image.data;

          addTexture(texture, textureMsg.texture_index);
        }
        else
        {
          RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Failed texture service call with index %d!", material.texture_index);
        }
      }
    }
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Failed materials service call!");
  }
}

void MeshDisplay::cacheVertexCosts(std::string layer, const std::vector<float>& costs)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Cache vertex cost map '" << layer << "' for UUID ");

  // insert into cache
  auto it = m_costCache.find(layer);
  if (it != m_costCache.end())
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "The cost layer \"" << layer << "\" has been updated.");
    m_costCache.erase(layer);
  }
  else
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "The cost layer \"" << layer << "\" has been added.");
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

}  // End namespace rviz_mesh_tools_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mesh_tools_plugins::MeshDisplay, rviz_common::Display)