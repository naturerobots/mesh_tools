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
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 *    Malte kleine Piening <malte@klpiening.de>
 */

#ifndef MESH_DISPLAY_HPP
#define MESH_DISPLAY_HPP

#include <Types.hpp>
#include <MeshVisual.hpp>

#include <vector>
#include <memory>
#include <queue>

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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>

#include <tf2_ros/message_filter.h>

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
class RosTopicProperty;
class EnumProperty;
class StringProperty;

}  // End namespace rviz

namespace rviz_map_plugin
{
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

// Forward declaration
class MeshVisual;

/**
 * @class MeshDisplay
 * @brief Display for showing the mesh in different modes
 */
class MeshDisplay : public rviz::Display
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
   * @brief RViz callback on enable
   */
  void onEnable();

  /**
   * @brief RViz callback on disable
   */
  void onDisable();

  /**
   * @brief Set the topics to subscribe.
   */
  void subscribe();

  /**
   * @brief Unsubscribes all topics.
   */
  void unsubscribe();

  /**
   * @brief disables visualization of incoming messages
   *
   * When called, incoming mesh messages will be ignored and the rviz properties to set topics and services will be
   * hidden. This cloud be useful when using this display as a child-display e.g. of the MapDisplay
   */
  void ignoreIncomingMessages();

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
   * @brief Clears the vertex costs
   */
  void clearVertexCosts();

  /**
   * @brief Adds a vertex costlayer
   * @param costlayer Name of the new costlayer
   * @param vertexCosts The vertex costs
   */
  void addVertexCosts(std::string costlayer, vector<float>& vertexCosts);

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
   * @brief Set geometrys pose
   * @param position position of the pose
   * @param orientation orientation of the pose
   */
  void setPose(Ogre::Vector3& position, Ogre::Quaternion& orientation);

private Q_SLOTS:

  /**
   * @brief Updates the buffer size
   */
  void updateBufferSize();

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

  /**
   * @brief Update the color of the mesh normals
   */
  void updateNormalsColor();

  /**
   * @brief Update the size of the mesh normals
   */
  void updateNormalsSize();

  /**
   * @brief Update the vertex costs
   */
  void updateVertexCosts();

  /**
   * @brief Updates the subscribed vertex colors topic.
   */
  void updateVertexColorsTopic();

  /**
   * @brief Updates the subscribed vertex costs topic.
   */
  void updateVertexCostsTopic();

  /**
   * @brief Updates the subscribed topic.
   */
  void updateTopic();

  /**
   * @brief Updates the vertex color service.
   */
  void updateVertexColorService();

  /**
   * @brief Updates the material and texture services.
   */
  void updateMaterialAndTextureServices();

private:
  /**
   * @brief RViz callback on initialize
   */
  void onInitialize();

  /**
   * @brief initial service call for UUID & geometry
   */
  void initialServiceCall();

  /**
   * @brief Sets data for trianglemesh_visual and updates the mesh.
   * @param meshMsg Message containing geometry information
   */
  void processMessage(const mesh_msgs::MeshGeometryStamped::ConstPtr& meshMsg);

  /**
   * @brief Handler for incoming geometry messages. Validate data and update mesh
   * @param meshMsg The geometry
   */
  void incomingGeometry(const mesh_msgs::MeshGeometryStamped::ConstPtr& meshMsg);

  /**
   * @brief Handler for incoming vertex color messages. Validate data and update mesh
   * @param colorsStamped The vertex colors
   */
  void incomingVertexColors(const mesh_msgs::MeshVertexColorsStamped::ConstPtr& colorsStamped);

  /**
   * @brief Handler for incoming vertex cost messages. Validate data and update mesh
   * @param costsStamped The vertex costs
   */
  void incomingVertexCosts(const mesh_msgs::MeshVertexCostsStamped::ConstPtr& costsStamped);

  /**
   * @brief Requests vertex colors from the specified service
   * @param uuid Mesh UUID
   */
  void requestVertexColors(std::string uuid);

  /**
   * @brief Requests materials from the specified service
   * @param uuid Mesh UUID
   */
  void requestMaterials(std::string uuid);

  /**
   * @brief Cache function for vertex cost messages.
   * @param costsStamped The vertex cost message
   */
  void cacheVertexCosts(std::string layer, const std::vector<float>& costs);

  /**
   * @brief delivers the latest mesh visual
   * @return latest mesh visual
   */
  std::shared_ptr<MeshVisual> getLatestVisual();

  /**
   * @brief adds a new visual to the ring buffer
   * @return newly added visual
   */
  std::shared_ptr<MeshVisual> addNewVisual();

  /// if set to true, ignore incoming messages and do not use services to request materials
  bool m_ignoreMsgs;

  /// Client to request the vertex colors
  ros::ServiceClient m_vertexColorClient;

  /// Client to request the materials
  ros::ServiceClient m_materialsClient;

  /// Client to request the textures
  ros::ServiceClient m_textureClient;

  /// Client to request the UUID
  ros::ServiceClient m_uuidClient;

  /// Client to request the geometry
  ros::ServiceClient m_geometryClient;

  /// Subscriber for meshMsg
  message_filters::Subscriber<mesh_msgs::MeshGeometryStamped> m_meshSubscriber;

  /// Subscriber for vertex colors
  message_filters::Subscriber<mesh_msgs::MeshVertexColorsStamped> m_vertexColorsSubscriber;

  /// Subscriber for vertex costs
  message_filters::Subscriber<mesh_msgs::MeshVertexCostsStamped> m_vertexCostsSubscriber;

  /// Messagefilter for meshMsg
  tf2_ros::MessageFilter<mesh_msgs::MeshGeometryStamped>* m_tfMeshFilter;

  /// Messagefilter for vertex colors
  tf2_ros::MessageFilter<mesh_msgs::MeshVertexColorsStamped>* m_tfVertexColorsFilter;

  /// Messagefilter for vertex costs
  tf2_ros::MessageFilter<mesh_msgs::MeshVertexCostsStamped>* m_tfVertexCostsFilter;

  /// Synchronizer for meshMsgs
  message_filters::Cache<mesh_msgs::MeshGeometryStamped>* m_meshSynchronizer;

  /// Synchronizer for vertex colors
  message_filters::Cache<mesh_msgs::MeshVertexColorsStamped>* m_colorsSynchronizer;

  /// Synchronizer for vertex costs
  message_filters::Cache<mesh_msgs::MeshVertexCostsStamped>* m_costsSynchronizer;

  /// Counter for the received messages
  uint32_t m_messagesReceived;

  /// Uuid of the last received message
  std::string m_lastUuid;

  /// Visual data
  std::queue<std::shared_ptr<MeshVisual>> m_visuals;

  /// Property to handle topic for meshMsg
  rviz::RosTopicProperty* m_meshTopic;

  /// Property to handle buffer size
  rviz::IntProperty* m_bufferSize;

  /// Property to select the display type
  rviz::EnumProperty* m_displayType;

  /// Property to set faces color
  rviz::ColorProperty* m_facesColor;

  /// Property to set faces transparency
  rviz::FloatProperty* m_facesAlpha;

  /// Property to handle topic for vertex colors
  rviz::RosTopicProperty* m_vertexColorsTopic;

  /// Property to handle service name for vertexColors
  rviz::StringProperty* m_vertexColorServiceName;

  /// Property to only show textured faces when texturizing is enabled
  rviz::BoolProperty* m_showTexturedFacesOnly;

  /// Property to handle service name for materials
  rviz::StringProperty* m_materialServiceName;

  /// Property to handle service name for textures
  rviz::StringProperty* m_textureServiceName;

  /// Property for selecting the color type for cost display
  rviz::EnumProperty* m_costColorType;

  /// Property to handle topic for vertex cost maps
  rviz::RosTopicProperty* m_vertexCostsTopic;

  /// Property to select different types of vertex cost maps to be shown
  rviz::EnumProperty* m_selectVertexCostMap;

  /// Property for using custom limits for cost display
  rviz::BoolProperty* m_costUseCustomLimits;

  /// Property for setting the lower limit of cost display
  rviz::FloatProperty* m_costLowerLimit;

  /// Property for setting the upper limit of cost display
  rviz::FloatProperty* m_costUpperLimit;

  /// Property to select the normals
  rviz::BoolProperty* m_showNormals;

  /// Property to set the color of the normals
  rviz::ColorProperty* m_normalsColor;

  /// Property to set the transparency of the normals
  rviz::FloatProperty* m_normalsAlpha;

  /// Property to set the size of the normals
  rviz::FloatProperty* m_scalingFactor;

  /// Property to select the wireframe
  rviz::BoolProperty* m_showWireframe;

  /// Property to set wireframe color
  rviz::ColorProperty* m_wireframeColor;

  /// Property to set wireframe transparency
  rviz::FloatProperty* m_wireframeAlpha;

  /// Cache for received vertex cost messages
  std::map<std::string, std::vector<float>> m_costCache;
};

}  // end namespace rviz_map_plugin

#endif
