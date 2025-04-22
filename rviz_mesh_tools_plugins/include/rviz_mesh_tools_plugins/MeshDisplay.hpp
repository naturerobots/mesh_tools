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

#include <rviz_mesh_tools_plugins/Types.hpp>
#include <rviz_mesh_tools_plugins/MeshVisual.hpp>

#include <vector>
#include <memory>
#include <queue>

#include <string>
#include <math.h>
#include <algorithm>

#include <QMessageBox>
#include <QApplication>
#include <QIcon>
#include <QObject>


#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>


#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>

#include <rviz_mesh_tools_plugins/RVizMessageFilter.hpp>

#include <rviz_rendering/mesh_loader.hpp>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreStringConverter.h>
#include <OgreMaterialManager.h>
#include <OgreColourValue.h>

#include <mesh_msgs/srv/get_vertex_colors.hpp>
#include <mesh_msgs/srv/get_materials.hpp>
#include <mesh_msgs/srv/get_geometry.hpp>
#include <mesh_msgs/srv/get_texture.hpp>
#include <mesh_msgs/srv/get_uui_ds.hpp>

#endif // Q_MOC_RUN

#include "rclcpp/rclcpp.hpp"

namespace rviz_common
{
namespace properties
{
// Forward declaration
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
class QosProfileProperty;
class EnumProperty;
class StringProperty;
} // end namespace properties
} // end namespace rviz_common

namespace rviz_mesh_tools_plugins
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
class MeshDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  /**#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
   * @brief Constructor
   */
  MeshDisplay();

  /**
   * @brief Destructor
   */
  ~MeshDisplay();

  // virtual void load(const rviz_common::Config& config);

  /**
   * @brief Periodically called from rviz
   */
  void update(float wall_dt, float ros_dt) override;

  /**
   * @brief RViz callback on enable
   */
  void onEnable() override;

  /**
   * @brief RViz callback on disable
   */
  void onDisable() override;

  void reset() override;

  void fixedFrameChanged() override;

  /**
   * @brief Update all subscriptions. Individual subscription update function will check whether they are active. (e.g. vertex colors can also be inactive when the UI element is set to a fixed color).
   */
  void updateAllSubscriptions();

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
  void setGeometry(std::shared_ptr<Geometry> geometry);

  /**
   * @brief Set the reference frame of the map
   * @param frame the frame to set
   */
  void setMapFrame(const std::string& frame);

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
  void transformerChangedCallback();

  /**
   * @brief Updates the buffer size
   */
  void updateBufferSize();

  /**
   * @brief Updates the mesh material (how faces are colored)
   */
  void updateMeshMaterial();

  /**
   * @brief Updates UI elements that change the mesh display type
   */
  void updateDisplayType();

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

  //! Updates the subscription for getting the mesh geometry
  void updateMeshGeometrySubscription();

  /**
   * @brief Updates the subscribed vertex colors topic.
   */
  void updateVertexColorsSubscription();

  /**
   * @brief Updates the subscribed vertex costs topic.
   */
  void updateVertexCostsSubscription();

  /**
   * @brief Updates the vertex color service.
   */
  void updateVertexColorService();

  /**
   * @brief Updates the material and texture services.
   */
  void updateMaterialAndTextureServices();

  /**
   * @brief Updates the hardware culling mode of the mesh
   */
  void updateCullingMode();

private:
  /**
   * @brief RViz callback on initialize
   */
  void onInitialize() override;

  /**
   * @brief Update the transform to the map
   */
  void transformMesh();

  /**
   * @brief initial service call for UUID & geometry
   */
  void initialServiceCall();

  /**
   * @brief Sets data for trianglemesh_visual and updates the mesh.
   * @param meshMsg Message containing geometry information
   */
  void processMessage(const mesh_msgs::msg::MeshGeometryStamped& meshMsg);

  /**
   * @brief Handler for incoming geometry messages. Validate data and update mesh
   * @param meshMsg The geometry
   */
  void meshGeometryCallback(const mesh_msgs::msg::MeshGeometryStamped::ConstSharedPtr& meshMsg);

  /**
   * @brief Handler for incoming vertex color messages. Validate data and update mesh
   * @param colorsStamped The vertex colors
   */
  void vertexColorsCallback(const mesh_msgs::msg::MeshVertexColorsStamped::ConstSharedPtr& colorsStamped);

  /**
   * @brief Handler for incoming vertex cost messages. Validate data and update mesh
   * @param costsStamped The vertex costs
   */
  void vertexCostsCallback(const mesh_msgs::msg::MeshVertexCostsStamped::ConstSharedPtr& costsStamped);

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
  rclcpp::Client<mesh_msgs::srv::GetVertexColors>::SharedPtr m_vertexColorClient;
  /// Client to request the materials
  rclcpp::Client<mesh_msgs::srv::GetMaterials>::SharedPtr m_materialsClient;
  /// Client to request the textures
  rclcpp::Client<mesh_msgs::srv::GetTexture>::SharedPtr m_textureClient;
  /// Client to request the UUID
  rclcpp::Client<mesh_msgs::srv::GetUUIDs>::SharedPtr m_uuidClient;
  /// Client to request the geometry
  rclcpp::Client<mesh_msgs::srv::GetGeometry>::SharedPtr m_geometryClient;

  /// Subscriber for meshMsg
  message_filters::Subscriber<mesh_msgs::msg::MeshGeometryStamped> m_meshSubscriber;
  /// Subscriber for vertex colors
  message_filters::Subscriber<mesh_msgs::msg::MeshVertexColorsStamped> m_vertexColorsSubscriber;
  /// Subscriber for vertex costs
  message_filters::Subscriber<mesh_msgs::msg::MeshVertexCostsStamped> m_vertexCostsSubscriber;

  /// TF2 message filter for incoming mesh data. Ensures we only process meshes for which a suitable TF is available
  tf2_ros::RVizMessageFilterPtr<mesh_msgs::msg::MeshGeometryStamped> m_tfMeshFilter;

  /// Cache for vertex colors, useful for when color information arrives before the mesh geometry
  std::shared_ptr<message_filters::Cache<mesh_msgs::msg::MeshVertexColorsStamped>> m_colorsMsgCache;
  /// Cache for vertex costs, useful for when cost information arrives before the mesh geometry
  std::shared_ptr<message_filters::Cache<mesh_msgs::msg::MeshVertexCostsStamped>> m_costsMsgCache;

  /// Counter for the received messages
  uint32_t m_messagesReceived;
  /// Uuid of the last received message
  std::string m_lastUuid;
  /// Visual data
  std::queue<std::shared_ptr<MeshVisual>> m_visuals;

  /// TF Frame of the last received message
  std::string m_meshFrame;

  // ================= UI members =================

  /// Properties to handle topic for meshMsg
  rviz_common::properties::RosTopicProperty* m_meshTopic;
  rviz_common::properties::QosProfileProperty* m_meshTopicQos;
  rclcpp::QoS m_meshQos;

  /// Property to handle buffer size
  rviz_common::properties::IntProperty* m_bufferSize;

  /// Property to select the display type
  rviz_common::properties::EnumProperty* m_displayType;

  /// Property to set faces color
  rviz_common::properties::ColorProperty* m_facesColor;

  /// Property to set faces transparency
  rviz_common::properties::FloatProperty* m_facesAlpha;

  /// Properties to handle topic for vertex colors
  rviz_common::properties::RosTopicProperty* m_vertexColorsTopic;
  rviz_common::properties::QosProfileProperty* m_vertexColorsTopicQos;
  rclcpp::QoS m_vertexColorsQos;

  /// Property to handle service name for vertexColors
  rviz_common::properties::StringProperty* m_vertexColorServiceName;

  /// Property to only show textured faces when texturizing is enabled
  rviz_common::properties::BoolProperty* m_showTexturedFacesOnly;

  /// Property to handle service name for materials
  rviz_common::properties::StringProperty* m_materialServiceName;

  /// Property to handle service name for textures
  rviz_common::properties::StringProperty* m_textureServiceName;

  /// Property for selecting the color type for cost display
  rviz_common::properties::EnumProperty* m_costColorType;

  /// Properties to handle topic for vertex cost maps
  rviz_common::properties::RosTopicProperty* m_vertexCostsTopic;
  rviz_common::properties::QosProfileProperty* m_vertexCostsTopicQos;
  rclcpp::QoS m_vertexCostsQos;

  /// Property to select different types of vertex cost maps to be shown
  rviz_common::properties::EnumProperty* m_selectVertexCostMap;

  /// Property for using custom limits for cost display
  rviz_common::properties::BoolProperty* m_costUseCustomLimits;

  /// Property for setting the lower limit of cost display
  rviz_common::properties::FloatProperty* m_costLowerLimit;

  /// Property for setting the upper limit of cost display
  rviz_common::properties::FloatProperty* m_costUpperLimit;

  /// Property to select the normals
  rviz_common::properties::BoolProperty* m_showNormals;

  /// Property to set the color of the normals
  rviz_common::properties::ColorProperty* m_normalsColor;

  /// Property to set the transparency of the normals
  rviz_common::properties::FloatProperty* m_normalsAlpha;

  /// Property to set the size of the normals
  rviz_common::properties::FloatProperty* m_scalingFactor;

  /// Property to select the wireframe
  rviz_common::properties::BoolProperty* m_showWireframe;

  /// Property to set wireframe color
  rviz_common::properties::ColorProperty* m_wireframeColor;

  /// Property to set wireframe transparency
  rviz_common::properties::FloatProperty* m_wireframeAlpha;

  /// Property to select the hardware culling mode
  rviz_common::properties::EnumProperty* m_cullingMode;

  /// Cache for received vertex cost messages
  std::map<std::string, std::vector<float>> m_costCache;
};

}  // end namespace rviz_mesh_tools_plugins

#endif
