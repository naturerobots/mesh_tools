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
 *  ClusterLabelTool.cpp
 *
 *  authors:
 *            Kristin Schmidt <krschmidt@uos.de>
 *            Jan Philipp Vogtherr <jvogtherr@uos.de>
 *
 */

#include <algorithm>
#include <rviz_mesh_tools_plugins/ClusterLabelTool.hpp>
#include <rviz_mesh_tools_plugins/ClusterLabelVisual.hpp>
#include <rviz_mesh_tools_plugins/ClusterLabelDisplay.hpp>
#include <rviz_mesh_tools_plugins/InteractionHelper.hpp>

#include <fstream>
#include <sstream>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/interaction/selection_manager_iface.hpp>
#include <rviz_common/view_manager.hpp>

#include <rviz_rendering/render_window.hpp>

#include <OgreColourValue.h>
#include <OgreCamera.h>
#include <OgreTechnique.h>
#include <OgreViewport.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRenderTexture.h>
#include <OgreImage.h>

#include <ament_index_cpp/get_package_share_directory.hpp>



using std::ifstream;
using std::stringstream;

namespace rviz_mesh_tools_plugins
{

ClusterLabelTool::ClusterLabelTool() 
:rviz_common::Tool()
, m_display(nullptr)
, m_selectionCircle(nullptr)
, m_selectionMesh(nullptr)
, m_selectionVisibilityBit(0)
{
  shortcut_key_ = 'l';
}

ClusterLabelTool::~ClusterLabelTool()
{
  m_selectedFaces.clear();
  if (m_selectionBox)
  {
    scene_manager_->destroyManualObject(m_selectionBox->getName());
    m_selectionBox = nullptr;
  }
  if (m_selectionBoxMaterial)
  {
    scene_manager_->destroyManualObject(m_selectionBoxMaterial->getName());
    m_selectionBoxMaterial.reset();
  }
  if (m_selectionCircle)
  {
    scene_manager_->destroyManualObject(m_selectionCircle);
    m_selectionCircle = nullptr;
  }
  if (m_selectionMesh)
  {
    scene_manager_->destroyManualObject(m_selectionMesh);
    m_selectionMesh = nullptr;
  }
  if (m_sceneNode)
  {
    scene_manager_->destroySceneNode(m_sceneNode);
    m_sceneNode = nullptr;
  }
  if (m_selectionSceneNode)
  {
    scene_manager_->destroySceneNode(m_selectionSceneNode);
    m_selectionSceneNode = nullptr;
  }
  if (m_selectionCircleNode)
  {
    scene_manager_->destroySceneNode(m_selectionCircleNode);
    m_selectionCircleNode = nullptr;
  }
  context_->visibilityBits()->freeBits(m_selectionVisibilityBit);
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
void ClusterLabelTool::onInitialize()
{
  RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "ClusterLabelTool: Call Init");
  
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  m_labelPublisher = node->create_publisher<mesh_msgs::msg::MeshFaceClusterStamped>(
    "/cluster_label", 1);

  m_sceneNode = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();

  m_selectionBox = context_->getSceneManager()->createManualObject("ClusterLabelTool_SelectionBox");
  m_selectionBox->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  m_selectionBox->setUseIdentityProjection(true);
  m_selectionBox->setUseIdentityView(true);
  m_selectionBox->setQueryFlags(0);
  m_selectionBox->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  m_sceneNode->attachObject(m_selectionBox);

  m_selectionBoxMaterial = Ogre::MaterialManager::getSingleton().getByName(
    "ClusterLabelTool_SelectionBoxMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  if(!m_selectionBoxMaterial)
  {
    m_selectionBoxMaterial = Ogre::MaterialManager::getSingleton().create(
      "ClusterLabelTool_SelectionBoxMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
  }
  
  m_selectionBoxMaterial->setAmbient(Ogre::ColourValue(0, 0, 255, 0.5));
  m_selectionBoxMaterial->setDiffuse(0, 0, 0, 0.5);
  m_selectionBoxMaterial->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  m_selectionBoxMaterial->setDepthWriteEnabled(false);
  m_selectionBoxMaterial->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_SOLID);
  m_selectionBoxMaterial->setCullingMode(Ogre::CULL_NONE);

  // Circle overlay for the circular selection
  // The SceneNode is used to move and scale the circle
  m_selectionCircleNode = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  initSelectionCircle();

  m_selectionVisibilityBit = context_->visibilityBits()->allocBit();
  // Setup texture for accelerated picking
  // We render the scene in a 1280x720 resolution
  // There should only ever be one instance of this Tool so we can reuse the texture
  m_selectionTexture = Ogre::TextureManager::getSingleton().getByName("ClusterLabelToolPickRenderTex");
  if (nullptr == m_selectionTexture)
  {
    m_selectionTexture = Ogre::TextureManager::getSingleton().createManual(
      "ClusterLabelToolPickRenderTex",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      1280, 720,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET
    );
  }

  // Create a material
  m_selectionMaterial = Ogre::MaterialManager::getSingleton().getByName("ClusterLabelToolSelectionMat");
  if (nullptr == m_selectionMaterial)
  {
    m_selectionMaterial = Ogre::MaterialManager::getSingleton().create(
      "ClusterLabelToolSelectionMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true
    );
  }

  Ogre::Technique* tech = m_selectionMaterial->getTechnique(0);
  Ogre::Pass* pass = tech->getPass(0);
  pass->setLightingEnabled(false);
  pass->setAmbient(1, 1, 1);

  // Scene Node for our modified mesh
  m_selectionSceneNode = scene_manager_->getSceneNode("ClusterLabelToolSelection", false);
  if (nullptr == m_selectionSceneNode)
  {
    m_selectionSceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode("ClusterLabelToolSelection");
  }
}

void ClusterLabelTool::setVisual(std::shared_ptr<ClusterLabelVisual> visual)
{
  // set new visual
  m_visual = visual;
  m_selectedFaces = visual->getFaces();

  // Reset the selected faces
  std::fill(m_faceSelectedArray.begin(), m_faceSelectedArray.end(), false);
  for (auto faceId : m_selectedFaces)
  {
    if (m_faceSelectedArray.size() <= faceId)
    {
      // The faceSelectedArray has space for all faces
      // This is an error from the Visual
      continue;
    }
    m_faceSelectedArray[faceId] = true;
  }
}

void ClusterLabelTool::setBrushSize(float size)
{
  // m_clKernelSphere.setArg(3, size);
  m_brushSize = size;
}

void ClusterLabelTool::activate()
{
}

void ClusterLabelTool::deactivate()
{
}

void ClusterLabelTool::setDisplay(ClusterLabelDisplay* display)
{
  RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "ClusterLabelTool::setDisplay()");
  m_display = display;
  m_meshGeometry = m_display->getGeometry();
  m_faceSelectedArray.resize(m_meshGeometry->faces.size());
  std::fill(m_faceSelectedArray.begin(), m_faceSelectedArray.end(), false);

  // Prepare for GPU accelerated selection
  updateSelectionMesh();
}

void ClusterLabelTool::updateSelectionBox()
{
  // TODO CHECK THIS

  // left = m_evt_start.x * 2 - 1;
  // right = m_evt_stop.x * 2 - 1;
  // top = 1 - m_evt_start.y * 2;
  // bottom = 1 - m_evt_stop.y * 2;

  auto viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(m_evt_panel->getRenderWindow());
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  float m_bb_x1_rel = static_cast<float>(m_bb_x1) / static_cast<float>(width);
  float m_bb_x2_rel = static_cast<float>(m_bb_x2) / static_cast<float>(width);
  float m_bb_y1_rel = static_cast<float>(m_bb_y1) / static_cast<float>(height);
  float m_bb_y2_rel = static_cast<float>(m_bb_y2) / static_cast<float>(height);

  float left   = m_bb_x1_rel * 2.0 - 1.0;
  float right  = m_bb_x2_rel * 2.0 - 1.0;
  float top    = 1.0 - m_bb_y1_rel * 2.0;
  float bottom = 1.0 - m_bb_y2_rel * 2.0;

  m_selectionBox->clear();
  m_selectionBox->begin(m_selectionBoxMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  m_selectionBox->position(left, top, 0.0);
  m_selectionBox->position(right, top, 0.0);
  m_selectionBox->position(right, bottom, 0.0);
  m_selectionBox->position(left, bottom, 0.0);
  m_selectionBox->triangle(0, 1, 2);
  m_selectionBox->triangle(0, 2, 3);
  m_selectionBox->end();


}

void ClusterLabelTool::selectionBoxStart(rviz_common::ViewportMouseEvent& event)
{
  m_bb_x1 = event.x;
  m_bb_y1 = event.y;
  m_evt_panel = event.panel;

  m_bb_x2 = m_bb_x1;
  m_bb_y2 = m_bb_y1;
  m_selectionBox->clear();
  m_selectionBox->setVisible(true);
}

void ClusterLabelTool::selectionBoxMove(rviz_common::ViewportMouseEvent& event)
{
  m_bb_x2 = event.x;
  m_bb_y2 = event.y;
  m_evt_panel = event.panel;

  updateSelectionBox();
}

void ClusterLabelTool::selectMultipleFaces(
  rviz_common::ViewportMouseEvent& event, 
  bool selectMode)
{
  (void) event;
  // No map loaded or ClusterLabel display is not active
  if (!m_visual)
  {
    return;
  }


  Ogre::Image img = renderMeshWithFaceID();

  // Translate the selection box to uv coordinates
  uint32_t x0 = std::min(m_bb_x1, m_bb_x2);
  uint32_t x1 = std::max(m_bb_x1, m_bb_x2);
  uint32_t y0 = std::min(m_bb_y1, m_bb_y2);
  uint32_t y1 = std::max(m_bb_y1, m_bb_y2);

  const float u_min = std::clamp((float) x0 / m_evt_panel->width(), 0.0f, 1.0f);
  const float u_max = std::clamp((float) x1 / m_evt_panel->width(), 0.0f, 1.0f);
  const float v_min = std::clamp((float) y0 / m_evt_panel->height(), 0.0f, 1.0f);
  const float v_max = std::clamp((float) y1 / m_evt_panel->height(), 0.0f, 1.0f);

  RCLCPP_DEBUG(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "Selected rect uv: (%f, %f) (%f, %f)",
    u_min, v_min, u_max, v_max
  );

  // Translate to image coordinates
  const uint32_t tex_x0 = u_min * img.getWidth();
  const uint32_t tex_x1 = u_max * img.getWidth();
  const uint32_t tex_y0 = v_min * img.getHeight();
  const uint32_t tex_y1 = v_max * img.getHeight();

  RCLCPP_DEBUG(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "Selected rect tex: (%u, %u) (%u, %u)",
    tex_x0, tex_y0, tex_x1, tex_y1
  );

  // Update all faces in the selected region
  for (uint32_t y = tex_y0; y < tex_y1; y++)
  {
    for (uint32_t x = tex_x0; x < tex_x1; x++)
    {
      Ogre::ColourValue color = img.getColourAt(x, y, 0);
      color.a = 0.0;
      uint32_t face_id = color.getAsARGB();
      if (face_id >= m_faceSelectedArray.size())
      {
        // These are pixels which have the background color
        continue;
      }
      else
      {
        m_faceSelectedArray[face_id] = selectMode;
      }
    }
  }

  // Update the visual with the selected faces
  // TODO: How to do this without creating the tmpFaceList all the time
  // Maybe use a std::set?
  std::vector<uint32_t> tmpFaceList;
  for(size_t faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
  {
    if (m_faceSelectedArray[faceId])
    {
      tmpFaceList.push_back(faceId);
    }
  }

  // Maybe the user has not loaded a map yet?
  if (m_visual)
  {
    m_visual->setFacesInCluster(tmpFaceList);
  }
}


void ClusterLabelTool::selectSingleFace(
  rviz_common::ViewportMouseEvent& event,
  bool selectMode)
{
  
  Ogre::Ray mouse_ray = getMouseEventRay(event);

  Intersection intersection;
  if(selectFace(context_, mouse_ray, intersection))
  {
    // std::cout << "selectSingleFace- HIT!" << std::endl;

    if (m_display && m_visual)
    {
      
      std::vector<uint32_t> tmpFaceList;
      if(m_faceSelectedArray.size() <= intersection.face_id)
      {
        // TODO: what is this? wtf
        m_faceSelectedArray.resize(intersection.face_id + 1);
      }
      m_faceSelectedArray[intersection.face_id] = selectMode;

      for(size_t faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
      {
        if (m_faceSelectedArray[faceId])
        {
          tmpFaceList.push_back(faceId);
        }
      }

      m_visual->setFacesInCluster(tmpFaceList);
      RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "selectSingleFaceParallel() found face with id %d", intersection.face_id);
    }

  } else {
    // std::cout << "selectSingleFace - No hit :(" << std::endl;
  }
}


void ClusterLabelTool::selectCircleFaces(
  rviz_common::ViewportMouseEvent& event, bool selectMode)
{
  // No map loaded or ClusterLabel display is not active
  if (!m_visual)
  {
    return;
  }

  // Get the selection buffer
  Ogre::Image buffer = renderMeshWithFaceID();

  // The diameter of the selection circle in RViz Display pixels
  // This needs to be mapped using uv coordinates because the height and width
  // if the selection buffer image are fixed.
  const float& diameter = m_brushSize;
  const float du = (float) diameter / event.panel->width();
  const float dv = (float) diameter / event.panel->height();

  // Position of the curser in uv coordinates
  const float u = (float) event.x / event.panel->width();
  const float v = (float) event.y / event.panel->height();

  // Position of the curser in img coordinates
  const int32_t cx = u * buffer.getWidth();
  const int32_t cy = v * buffer.getHeight();

  // X/Y Diameter in img coordinates
  const int32_t dx = du * buffer.getWidth();
  const int32_t dy = dv * buffer.getHeight();

  // Bounding rect in img coordinates
  const uint32_t x_min = std::clamp<int32_t>(cx - dx / 2, 0, buffer.getWidth());
  const uint32_t x_max = std::clamp<int32_t>(cx + dx / 2, 0, buffer.getWidth());
  const uint32_t y_min = std::clamp<int32_t>(cy - dy / 2, 0, buffer.getHeight());
  const uint32_t y_max = std::clamp<int32_t>(cy + dy / 2, 0, buffer.getHeight());

  const int32_t rx = dx / 2;
  const int32_t ry = dy / 2;
  for (uint32_t y = y_min; y < y_max; y++)
  {
    for (uint32_t x = x_min; x < x_max; x++)
    {
      // Check if the point lies inside the img space ellipse
      const int32_t dx = x - cx;
      const int32_t dy = y - cy;

      const float t = (std::pow(dx, 2) / std::pow(rx, 2)) + (std::pow(dy, 2) / std::pow(ry, 2));
      if (t >= 1.0)
      {
        continue;
      }

      // Point is inside the ellipse
      Ogre::ColourValue color = buffer.getColourAt(x, y, 0);
      color.a = 0.0;
      const uint32_t face_id = color.getAsARGB();

      // Background color is white and results in an invalid face_id
      if (face_id >= m_faceSelectedArray.size())
      {
        continue;
      }

      m_faceSelectedArray[face_id] = selectMode;
    }
  }

  // Update the visual
  std::vector<uint32_t> tmpFaceList;
  for(size_t faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
  {
    if (m_faceSelectedArray[faceId])
    {
      tmpFaceList.push_back(faceId);
    }
  }

  m_visual->setFacesInCluster(tmpFaceList);
}


void ClusterLabelTool::publishLabel(std::string label)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Label Tool: Publish label '" << label << "'");

  vector<uint32_t> faces;
  for (uint32_t i = 0; i < m_faceSelectedArray.size(); i++)
  {
    if (m_faceSelectedArray[i])
      faces.push_back(i);
  }

  m_display->addLabel(label, faces);
}

// Handling mouse event and mark the clicked faces
int ClusterLabelTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  /* 
   * Left Mouse is to select something, Right Mouse is to deselect something.
   *
   * Modifiers:
   *
   * None: Mark all faces inside a circle around the current mouse position
   * Shift: Create a rectangle and mark all faces inside on mouse up
   */

  // Show circle overlay when control is pressed
  if (event.control())
  {
    // Update the brush size with the wheel delta
    // In my tests one wheel step was always +-120
    if (event.wheel_delta > 0)
    {
      m_brushSize = std::max(MIN_BRUSH_SIZE, m_brushSize + MOUSE_WHEEL_BRUSH_SIZE_STEP);
    }
    else if (event.wheel_delta < 0)
    {
      m_brushSize = std::max(MIN_BRUSH_SIZE, m_brushSize - MOUSE_WHEEL_BRUSH_SIZE_STEP);
    }

    updateSelectionCircle(event);
    m_selectionCircle->setVisible(true);
  }
  else
  {
    m_selectionCircle->setVisible(false);
  }


  if (event.leftDown() && event.shift())
  {
    m_multipleSelect = true;
    selectionBoxStart(event);
  }
  else if (event.leftDown() && event.control())
  {
    m_circleSelect = true;
    m_selectionMode = true;
    selectCircleFaces(event, true);
  }
  else if (event.leftDown()) // Option without modifier needs to come last
  {
    m_singleSelect = true;
    selectSingleFace(event, true);
  }
  else if (event.leftUp() && m_singleSelect)
  {
    m_singleSelect = false;
  }
  else if (event.leftUp() && m_multipleSelect)
  {
    m_multipleSelect = false;
    m_selectionBox->setVisible(false);
    selectMultipleFaces(event, true);
  }
  else if (event.leftUp() && m_circleSelect)
  {
    m_circleSelect = false;
  }
  else if (event.rightDown() && event.shift())
  {
    m_multipleSelect = true;
    selectionBoxStart(event);
  }
  else if (event.rightDown() && event.control())
  {
    m_circleSelect = true;
    m_selectionMode = false;
    selectCircleFaces(event, false);
  }
  else if (event.rightDown())
  {
    m_singleSelect = true;
    selectSingleFace(event, false);
  }
  else if (event.rightUp() && m_singleSelect)
  {
    m_singleSelect = false;
  }
  else if (event.rightUp() && m_multipleSelect)
  {
    m_multipleSelect = false;
    m_selectionBox->setVisible(false);
    selectMultipleFaces(event, false);
  }
  else if (event.rightUp() && m_circleSelect)
  {
    m_circleSelect = false;
  }
  else if (m_multipleSelect)
  {
    selectionBoxMove(event);
  }
  else if (m_circleSelect)
  {
    selectCircleFaces(event, m_selectionMode);
  }

  return Render;
}

std::vector<uint32_t> ClusterLabelTool::getSelectedFaces()
{
  std::vector<uint32_t> faceList;

  for (uint32_t faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
  {
    if (m_faceSelectedArray[faceId])
    {
      faceList.push_back(faceId);
    }
  }
  return faceList;
}

void ClusterLabelTool::resetFaces()
{
  m_faceSelectedArray.clear();
  if(m_visual)
  {
    m_visual->setFacesInCluster(std::vector<uint32_t>());
  }
}

void ClusterLabelTool::resetVisual()
{
  m_visual.reset();
}

Ogre::Image ClusterLabelTool::renderMeshWithFaceID()
{
  // Attach the mesh to scene node
  m_selectionSceneNode->attachObject(m_selectionMesh);

  // Get the current RViz camera
  Ogre::Camera* camera = context_->getViewManager()->getCurrent()->getCamera();

  // Setup the render target to be the texture
  Ogre::RenderTexture* render_texture = m_selectionTexture->getBuffer()->getRenderTarget();
  Ogre::Viewport* viewport = render_texture->addViewport(camera);
  viewport->setClearEveryFrame(true);
  viewport->setOverlaysEnabled(false);
  viewport->setBackgroundColour(Ogre::ColourValue::White);
  viewport->setVisibilityMask(m_selectionVisibilityBit);

  // Actually render offscreen
  render_texture->update();

  // Copy the data to ram
  Ogre::Image img(Ogre::PF_R8G8B8, render_texture->getWidth(), render_texture->getHeight());
  Ogre::PixelBox pixels = img.getPixelBox();
  render_texture->copyContentsToMemory(pixels, pixels);

  // Detach the rviz camera
  render_texture->removeAllViewports();
  // Prevent our copied Mesh to be visible in the RViz Camera
  m_selectionSceneNode->detachAllObjects();

  return img;
}


void ClusterLabelTool::updateSelectionMesh()
{
  // For GPU accelerated picking we need per face attributes in the shader.
  // Sadly Ogre 1.12 does not have a way to do this.
  // Solution: Create a renderable geometry with 3 vertices per face and
  // add a vertex property with the original face_id
  // Then we can write a shader to write the face id to rendered pixels.
  if (nullptr == m_selectionMesh)
  {
    // Init the ManualObject used for offscreen rendering
    m_selectionMesh = scene_manager_->createManualObject("ClusterLabelToolPickingMesh");
    m_selectionMesh->estimateVertexCount(m_meshGeometry->faces.size() * 3);
    m_selectionMesh->estimateIndexCount(m_meshGeometry->faces.size() * 3);
    m_selectionMesh->begin(m_selectionBoxMaterial, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  }
  else
  {
    m_selectionMesh->estimateVertexCount(m_meshGeometry->faces.size() * 3);
    m_selectionMesh->estimateIndexCount(m_meshGeometry->faces.size() * 3);
    m_selectionMesh->beginUpdate(0);
  }


  // We can only use RGB (24) bits to to represent triangle id
  const uint32_t MAX_NUM_FACES = std::pow<uint32_t>(2, 24);
  if (MAX_NUM_FACES <= m_meshGeometry->faces.size())
  {
    RCLCPP_WARN(
      rclcpp::get_logger("rviz_mesh_tools_plugins"),
      "ClusterLabelTool: Mesh to label has more than the supported number of triangles (%u). This can lead to faces not being selected!",
      MAX_NUM_FACES
    );
  }

  for (uint32_t faceID = 0; faceID < m_meshGeometry->faces.size(); faceID++)
  {
    const auto& face = m_meshGeometry->faces[faceID];
    for (int i = 0; i < 3; i++)
    {
      const auto& v = m_meshGeometry->vertices[face.vertexIndices[i]];
      m_selectionMesh->position(v.x, v.y, v.z);
      Ogre::ColourValue color;
      color.setAsARGB(faceID);
      color.a = 1.0;
      m_selectionMesh->colour(color);
    }
    m_selectionMesh->triangle(faceID * 3 + 0, faceID * 3 + 1, faceID * 3 + 2);
  }
  m_selectionMesh->end();
  m_selectionMesh->setVisibilityFlags(m_selectionVisibilityBit);
  m_selectionMesh->setMaterial(0, m_selectionMaterial);

  // Remove the selection node from the previous parent
  if (auto* parent = m_selectionSceneNode->getParentSceneNode())
  {
    parent->removeChild(m_selectionSceneNode);
  }

  // TODO: This could or should be MeshDisplays scene node instead of the ClusterLabelDisplays
  // Or the ClusterLabelDisplay also needs to honor the map to fixed frame transform

  // Add our scene node below the displays scene node
  const auto& c = m_display->getSceneNode()->getChildren();
  if (c.end() == std::find(c.begin(), c.end(), m_selectionSceneNode))
  {
    m_display->getSceneNode()->addChild(m_selectionSceneNode);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "ClusterLabelTool: Load Mesh with %lu faces", m_meshGeometry->faces.size()
  );
}

void ClusterLabelTool::updateSelectionCircle(rviz_common::ViewportMouseEvent& event)
{
    const float x = (((float) event.x / event.panel->width()) * 2.0f) - 1.0f;
    const float y = (((float) event.y / event.panel->height()) * 2.0f) - 1.0f;

    // Scaling the circle to have the wanted dimensions in pixel space
    const float sx = ((float) m_brushSize / event.panel->width());
    const float sy = ((float) m_brushSize / event.panel->height());

    m_selectionCircleNode->setPosition(x, -y, 0.0f);
    m_selectionCircleNode->setScale({sx, sy, 1.0});
}


void ClusterLabelTool::initSelectionCircle()
{
  // Already initialized
  if (m_selectionCircle)
  {
    return;
  }

  m_selectionCircle = context_->getSceneManager()->createManualObject("ClusterLabelTool_SelectionCircle");
  m_selectionCircle->begin(m_selectionBoxMaterial, Ogre::RenderOperation::OT_LINE_STRIP);
  m_selectionCircle->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  m_selectionCircle->setUseIdentityProjection(true);
  m_selectionCircle->setUseIdentityView(true);
  m_selectionCircle->setQueryFlags(0);
  m_selectionCircle->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  m_selectionCircle->setVisibilityFlags(context_->getDefaultVisibilityBit());
  m_selectionCircle->setVisible(false);
  m_selectionCircleNode->attachObject(m_selectionCircle);

  // Generate 2D circle
  for (uint32_t i = 0; i < 60; i++)
  {
    const float rad = ((float) i / (60 - 1)) * 2.0 * M_PIf;
    const float x = std::cos(rad);
    const float y = std::sin(rad);
    m_selectionCircle->position(x, y, 0.0);
  }

  m_selectionCircle->end();
  m_selectionCircle->setMaterial(0, m_selectionBoxMaterial);
}
}  // End namespace rviz_mesh_tools_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mesh_tools_plugins::ClusterLabelTool, rviz_common::Tool)
