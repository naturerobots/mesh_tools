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
 *  ClusterLabelTool.hpp
 *
 *  authors:
 *            Kristin Schmidt <krschmidt@uos.de>
 *            Jan Philipp Vogtherr <jvogtherr@uos.de>
 *
 */

#ifndef CLUSTER_LABEL_TOOL_HPP
#define CLUSTER_LABEL_TOOL_HPP



#include <rviz_mesh_tools_plugins/Types.hpp>

#include <vector>
#include <memory>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include <QMessageBox>
#include <QApplication>
#include <QIcon>
#include <QObject>
#include <QWidget>
#include <QFrame>


#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/geometry.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/display.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/display_group.hpp>

#include <mesh_msgs/msg/mesh_face_cluster_stamped.hpp>

#ifndef Q_MOC_RUN
#include <rviz_rendering/mesh_loader.hpp>


#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreStringConverter.h>
#include <OgreMaterialManager.h>
#include <OgreRay.h>
#include <OgreSceneQuery.h>

#include <mesh_msgs/msg/mesh_face_cluster_stamped.hpp>

#endif // Q_MOC_RUN

#include <rclcpp/rclcpp.hpp>

namespace rviz_common
{
namespace properties
{
class RosTopicProperty;
class ColorProperty;
} // namespace properties
} // namespace rviz_common

// OGRE stuff
namespace Ogre
{
// Forward declaration
// class Vector3;
}  // namespace Ogre

namespace rviz_mesh_tools_plugins
{
// Forward declarations
class ClusterLabelDisplay;
class ClusterLabelVisual;

/**
 * @class ClusterLabelTool
 * @brief Tool for selecting faces
 */
class ClusterLabelTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  // Constants
  static constexpr float MIN_BRUSH_SIZE = 20.0f;
  static constexpr float MOUSE_WHEEL_BRUSH_SIZE_STEP = 10;

  /**
   * @brief Constructor
   */
  ClusterLabelTool();

  /**
   * @brief Destructor
   */
  virtual ~ClusterLabelTool();

  /**
   * @brief RViz callback on initialize
   */
  virtual void onInitialize();

  /**
   * @brief RViz callback for activating
   */
  virtual void activate();

  /**
   * @bríef RViz callback for deactivating
   */
  virtual void deactivate();

  /**
   * @brief RViz callback for mouse events
   * @param event The mouse event
   * @return Exit code
   */
  virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event);

  /**
   * @brief Connects this tool with a given display
   * @param display The display that creates this tool
   */
  void setDisplay(ClusterLabelDisplay* display);

  /**
   * @brief Connects this tool with a given visual
   * @param visual The visual that will become editable with this tool
   */
  void setVisual(std::shared_ptr<ClusterLabelVisual> visual);

  /**
   * @brief Adjust the circle size for the brush tool
   * @param size The circle diameter in screen Pixels
   */
  void setBrushSize(float size);

  /**
   * @brief Set the culling mode for selection to match the MeshVisual
   */
  void setCullingMode(Ogre::CullingMode mode);

public Q_SLOTS:

  /**
   * @brief Publish a label with a given namen
   * @param name The label name
   */
  void publishLabel(std::string name);

  /**
   * @brief Returns a list of selected face ID's
   * @return List of face ID's
   */
  std::vector<uint32_t> getSelectedFaces();

  /**
   * @brief Resets the list of selected faces
   */
  void resetFaces();

  /**
   * @brief Resets the current visual
   */
  void resetVisual();

private:
  std::vector<uint32_t> m_selectedFaces;
  std::vector<bool> m_faceSelectedArray;
  ClusterLabelDisplay* m_display;
  std::shared_ptr<ClusterLabelVisual> m_visual;
  std::shared_ptr<Geometry> m_meshGeometry;
  float m_brushSize;

  // Selection Box
  Ogre::SceneNode* m_sceneNode;
  Ogre::ManualObject* m_selectionBox;
  Ogre::MaterialPtr m_selectionBoxMaterial;

  // Selection Circle
  Ogre::ManualObject* m_selectionCircle;
  Ogre::SceneNode* m_selectionCircleNode;

  int m_bb_x1;
  int m_bb_y1;
  int m_bb_x2;
  int m_bb_y2;

  rviz_common::RenderPanel* m_evt_panel;

  // Selection Modes
  bool m_multipleSelect = false;
  bool m_singleSelect = false;
  bool m_circleSelect = false;
  // Select = true Deselect = false
  bool m_selectionMode = false;

  std::vector<Ogre::Vector3> m_vertexPositions;

  void initSelectionCircle();
  void updateSelectionCircle(rviz_common::ViewportMouseEvent& event);
  void updateSelectionBox();
  void selectionBoxStart(rviz_common::ViewportMouseEvent& event);
  void selectionBoxMove(rviz_common::ViewportMouseEvent& event);
  void selectMultipleFaces(rviz_common::ViewportMouseEvent& event, bool selectMode);
  void selectSingleFace(rviz_common::ViewportMouseEvent& event, bool selectMode);
  void selectCircleFaces(rviz_common::ViewportMouseEvent& event, bool selectMode);

  rclcpp::Publisher<mesh_msgs::msg::MeshFaceClusterStamped>::SharedPtr m_labelPublisher;

  /**
   *  @brief Renders the current Mesh to an Offscreen Buffer using the FaceIDs as colors.
   *
   *  The resulting Image can be used to determine which faces are visible from the Camera.
   *  
   *  @return The rendered Image.
   */
  Ogre::Image renderMeshWithFaceID();

  /**
   *  @brief Setup the Selection Mesh from the current geometry
   */
  void updateSelectionMesh();

  // Accelerated area picking via Ogre render pass
  Ogre::TexturePtr m_selectionTexture;
  Ogre::MaterialPtr m_selectionMaterial;
  Ogre::ManualObject* m_selectionMesh;
  Ogre::SceneNode* m_selectionSceneNode;
  // Used to render only the selectionMesh to the offscreen Texture
  uint32_t m_selectionVisibilityBit;
  Ogre::CullingMode m_cullingMode;
};
}  // end namespace rviz_mesh_tools_plugins

#endif
