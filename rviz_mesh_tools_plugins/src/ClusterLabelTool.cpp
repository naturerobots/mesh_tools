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
#include <rviz_mesh_tools_plugins/CLUtil.hpp>

#include <rviz_mesh_tools_plugins/InteractionHelper.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>

#include <chrono>

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
// #define CL_RAY_CAST_KERNEL_FILE "/include/kernels/cast_rays.cl"



ClusterLabelTool::ClusterLabelTool() 
:rviz_common::Tool()
,m_displayInitialized(false)
// ,m_evt_start(nullptr, (QMouseEvent*)nullptr, 0, 0)
// ,m_evt_stop(nullptr, (QMouseEvent*)nullptr, 0, 0)
, m_selectionMesh(nullptr)
{
  shortcut_key_ = 'l';
}

ClusterLabelTool::~ClusterLabelTool()
{
  m_selectedFaces.clear();
  context_->getSceneManager()->destroyManualObject(m_selectionBox->getName());
  context_->getSceneManager()->destroyManualObject(m_selectionBoxMaterial->getName());
  context_->getSceneManager()->destroySceneNode(m_sceneNode);
  context_->getSceneManager()->destroySceneNode(m_selectionSceneNode);
  context_->getSceneManager()->destroyManualObject(m_selectionMesh);
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

  m_selectionVisibilityBit = context_->visibilityBits()->allocBit();

  // Setup texture for accelerated picking
  // We render the scene in a 1280x720 resolution
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
  m_faceSelectedArray.clear();
  for (auto faceId : m_selectedFaces)
  {
    if (m_faceSelectedArray.size() <= faceId)
    {
      m_faceSelectedArray.resize(faceId + 1);
    }
    m_faceSelectedArray[faceId] = true;
  }
}

void ClusterLabelTool::setSphereSize(float size)
{
  // m_clKernelSphere.setArg(3, size);
  m_sphereSize = size;
}

void ClusterLabelTool::activate()
{
}

void ClusterLabelTool::deactivate()
{
}

void ClusterLabelTool::setDisplay(ClusterLabelDisplay* display)
{
  m_display = display;
  m_meshGeometry = m_display->getGeometry();
  m_faceSelectedArray.reserve(m_meshGeometry->faces.size());
  m_displayInitialized = true;

  m_vertexData.reserve(m_meshGeometry->faces.size() * 3 * 3);

  for (uint32_t faceId = 0; faceId < m_meshGeometry->faces.size(); faceId++)
  {
    for (uint32_t i = 0; i < 3; i++)
    {
      uint32_t vertexId = m_meshGeometry->faces[faceId].vertexIndices[i];
      Ogre::Vector3 vertexPos(m_meshGeometry->vertices[vertexId].x, m_meshGeometry->vertices[vertexId].y,
                              m_meshGeometry->vertices[vertexId].z);
      m_vertexPositions.push_back(vertexPos);

      m_vertexData.push_back(m_meshGeometry->vertices[vertexId].x);
      m_vertexData.push_back(m_meshGeometry->vertices[vertexId].y);
      m_vertexData.push_back(m_meshGeometry->vertices[vertexId].z);
    }
  }

  // For GPU accelerated picking we need per face attributes in the shader.
  // Sadly Ogre 1.12 does not have a way to do this.
  // Solution: Create a renderable geometry with 3 vertices per face and
  // add a vertex property with the original face_id
  // Then we can write a shader to write the face id to rendered pixels.
  if (nullptr == m_selectionMesh)
  {
    m_selectionMesh = scene_manager_->createManualObject("ClusterLabelToolPickingMesh");
    m_selectionMesh->setDynamic(false);
    m_selectionMesh->begin(m_selectionBoxMaterial, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  }
  else
  {
    m_selectionMesh->beginUpdate(0);
  }

  // We can only use RGB (24) bits to to represent triangle id
  const uint32_t MAX_NUM_FACES = std::pow<uint32_t>(2, 24);
  if (MAX_NUM_FACES > m_meshGeometry->faces.size())
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

  if (auto* parent = m_selectionSceneNode->getParentSceneNode())
  {
    parent->removeChild(m_selectionSceneNode);
  }

  const auto& c = display->getSceneNode()->getChildren();
  if (c.end() == std::find(c.begin(), c.end(), m_selectionSceneNode))
  {
    display->getSceneNode()->addChild(m_selectionSceneNode);
  }
  m_selectionSceneNode->detachObject(m_selectionMesh);
  RCLCPP_INFO(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "ClusterLabelTool: Load Mesh with %lu faces", m_meshGeometry->faces.size()
  );
  m_faceSelectedArray.clear();
  m_faceSelectedArray.resize(m_meshGeometry->faces.size());
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
  m_selectionBox->position(left, top, -1);
  m_selectionBox->position(right, top, -1);
  m_selectionBox->position(right, bottom, -1);
  m_selectionBox->position(left, bottom, -1);
  m_selectionBox->triangle(0, 1, 2);
  m_selectionBox->triangle(0, 2, 3);
  m_selectionBox->end();


}

void ClusterLabelTool::selectionBoxStart(rviz_common::ViewportMouseEvent& event)
{
  // OLD: doesnt work anymore
  // m_selectionStart.x = (float)event.x / event.viewport->getActualWidth();
  // m_selectionStart.y = (float)event.y / event.viewport->getActualHeight();
  // NEW: 
  // TODO: Check if this is right
  // m_selectionStart.x = (float)event.x / event.panel->getRenderWindow()->width();
  // m_selectionStart.y = (float)event.y / event.panel->getRenderWindow()->height();

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
  // m_selectionStop.x = (float)event.x / event.viewport->getActualWidth();
  // m_selectionStop.y = (float)event.y / event.viewport->getActualHeight();
  
  // Not possible
  // m_selectionStop.x = (float)event.x / event.panel->getRenderWindow()->width();
  // m_selectionStop.y = (float)event.y / event.panel->getRenderWindow()->height();

  m_bb_x2 = event.x;
  m_bb_y2 = event.y;
  m_evt_panel = event.panel;

  updateSelectionBox();
}

void ClusterLabelTool::selectMultipleFaces(
  rviz_common::ViewportMouseEvent& event, 
  bool selectMode)
{
  m_selectionBox->setVisible(false);

  // Attach the mesh to scene node
  m_selectionSceneNode->attachObject(m_selectionMesh);

  // Get the current rviz camera
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
  // TODO: We could copy only the rect we are interested in?
  Ogre::Image img(Ogre::PF_R8G8B8, render_texture->getWidth(), render_texture->getHeight());
  Ogre::PixelBox pixels = img.getPixelBox();

  render_texture->copyContentsToMemory(pixels, pixels);

  // At this point we need to inspect the contents
  
  // Translate to uv coordinates
  uint32_t xl = std::min(m_bb_x1, m_bb_x2);
  uint32_t xr = std::max(m_bb_x1, m_bb_x2);
  uint32_t yl = std::min(m_bb_y1, m_bb_y2);
  uint32_t yr = std::max(m_bb_y1, m_bb_y2);

  const float u_min = std::clamp((float) xl / m_evt_panel->width(), 0.0f, 1.0f);
  const float u_max = std::clamp((float) xr / m_evt_panel->width(), 0.0f, 1.0f);
  const float v_min = std::clamp((float) yl / m_evt_panel->height(), 0.0f, 1.0f);
  const float v_max = std::clamp((float) yr / m_evt_panel->height(), 0.0f, 1.0f);

  RCLCPP_DEBUG(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "Selected rect uv: (%f, %f) (%f, %f)",
    u_min, v_min, u_max, v_max
  );

  const uint32_t tex_x0 = u_min * m_selectionTexture->getWidth();
  const uint32_t tex_x1 = u_max * m_selectionTexture->getWidth();
  const uint32_t tex_y0 = v_min * m_selectionTexture->getHeight();
  const uint32_t tex_y1 = v_max * m_selectionTexture->getHeight();

  RCLCPP_DEBUG(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "Selected rect tex: (%u, %u) (%u, %u)",
    tex_x0, tex_y0, tex_x1, tex_y1
  );

  img.save("test.png");
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
  m_selectionSceneNode->detachAllObjects();
  render_texture->removeAllViewports();

  // Update the visual with the selected faces
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

void ClusterLabelTool::selectFacesInBoxParallel(Ogre::PlaneBoundedVolume& volume, bool selectMode)
{
  m_boxData.clear();
  for (Ogre::Plane plane : volume.planes)
  {
    m_boxData.push_back(plane.normal.x);
    m_boxData.push_back(plane.normal.y);
    m_boxData.push_back(plane.normal.z);
    m_boxData.push_back(plane.d);
  }

  // try
  // {
  //   m_clQueue.enqueueWriteBuffer(m_clBoxBuffer, CL_TRUE, 0, sizeof(float) * 4 * 6, m_boxData.data());

  //   m_clQueue.enqueueNDRangeKernel(m_clKernelBox, cl::NullRange, cl::NDRange(m_meshGeometry->faces.size()),
  //                                  cl::NullRange, nullptr);
  //   m_clQueue.finish();

  //   m_resultDistances.resize(m_meshGeometry->faces.size());
  //   m_clQueue.enqueueReadBuffer(m_clResultBuffer, CL_TRUE, 0, sizeof(float) * m_meshGeometry->faces.size(),
  //                               m_resultDistances.data());
  // }
  // catch (cl::Error err)
  // {
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), err.what() << ": " << CLUtil::getErrorString(err.err()));
  //   RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "(" << CLUtil::getErrorDescription(err.err()) << ")");
  // }

  for (int faceId = 0; faceId < m_meshGeometry->faces.size(); faceId++)
  {
    if (m_resultDistances[faceId] > 0)
    {
      if (m_faceSelectedArray.size() <= faceId)
      {
        m_faceSelectedArray.resize(faceId + 1);
      }
      m_faceSelectedArray[faceId] = selectMode;
    }
  }

  std::vector<uint32_t> tmpFaceList;

  for (uint32_t faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
  {
    if (m_faceSelectedArray[faceId])
    {
      tmpFaceList.push_back(faceId);
    }
  }

  if (m_displayInitialized && m_visual)
  {
    m_visual->setFacesInCluster(tmpFaceList);
    // m_visual->setColor(Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f));
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

    if (m_displayInitialized && m_visual)
    {
      
      std::vector<uint32_t> tmpFaceList;
      if(m_faceSelectedArray.size() <= intersection.face_id)
      {
        // TODO: what is this? wtf
        m_faceSelectedArray.resize(intersection.face_id + 1);
      }
      m_faceSelectedArray[intersection.face_id] = selectMode;

      for(int faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
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

void ClusterLabelTool::selectSingleFaceParallel(Ogre::Ray& ray, bool selectMode)
{
  m_rayData = { ray.getOrigin().x,    ray.getOrigin().y,    ray.getOrigin().z,
                ray.getDirection().x, ray.getDirection().y, ray.getDirection().z };

  std::vector<std::pair<uint32_t, float>> intersectedFaceList;

  // try
  // {
  //   m_clQueue.enqueueWriteBuffer(m_clRayBuffer, CL_TRUE, 0, sizeof(float) * 6, m_rayData.data());

  //   m_clQueue.enqueueNDRangeKernel(m_clKernelSingleRay, cl::NullRange, cl::NDRange(m_meshGeometry->faces.size()),
  //                                  cl::NullRange, nullptr);
  //   m_clQueue.finish();

  //   m_resultDistances.resize(m_meshGeometry->faces.size());
  //   m_clQueue.enqueueReadBuffer(m_clResultBuffer, CL_TRUE, 0, sizeof(float) * m_meshGeometry->faces.size(),
  //                               m_resultDistances.data());
  // }
  // catch (cl::Error err)
  // {
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), err.what() << ": " << CLUtil::getErrorString(err.err()));
  //   RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "(" << CLUtil::getErrorDescription(err.err()) << ")");
  // }

  int closestFaceId = -1;
  float minDist = std::numeric_limits<float>::max();

  for (int i = 0; i < m_meshGeometry->faces.size(); i++)
  {
    if (m_resultDistances[i] > 0 && m_resultDistances[i] < minDist)
    {
      closestFaceId = i;
      minDist = m_resultDistances[i];
    }
  }

  if (m_displayInitialized && m_visual && closestFaceId != -1)
  {
    std::vector<uint32_t> tmpFaceList;

    if (m_faceSelectedArray.size() <= closestFaceId)
    {
      m_faceSelectedArray.resize(closestFaceId + 1);
    }
    m_faceSelectedArray[closestFaceId] = selectMode;

    for (int faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
    {
      if (m_faceSelectedArray[faceId])
      {
        tmpFaceList.push_back(faceId);
      }
    }

    m_visual->setFacesInCluster(tmpFaceList);
    RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "selectSingleFaceParallel() found face with id %d", closestFaceId);
  }
}

void ClusterLabelTool::selectSphereFaces(
  rviz_common::ViewportMouseEvent& event, bool selectMode)
{
  // Ogre::Ray ray = event.viewport->getCamera()->getCameraToViewportRay(
  //     (float)event.x / event.viewport->getActualWidth(), (float)event.y / event.viewport->getActualHeight());
  // Ogre::Ray ray = event.panel->getViewController()->getCamera()->getCameraToViewportRay(
  //   (float)event.x / event.panel->getRenderWindow()->width(),
  //   (float)event.y / event.panel->getRenderWindow()->height()
  // );
  throw std::runtime_error("TODO");
  Ogre::Ray ray;
  selectSphereFacesParallel(ray, selectMode);
}

void ClusterLabelTool::selectSphereFacesParallel(
  Ogre::Ray& ray, bool selectMode)
{
  auto raycastResult = getClosestIntersectedFaceParallel(ray);

  if (raycastResult)
  {
    Ogre::Vector3 sphereCenter = ray.getPoint(raycastResult->second);

    m_sphereData = { sphereCenter.x, sphereCenter.y, sphereCenter.z, raycastResult->second };

    // try
    // {
    //   m_clQueue.enqueueWriteBuffer(m_clSphereBuffer, CL_TRUE, 0, sizeof(float) * 4, m_sphereData.data());

    //   m_clQueue.enqueueNDRangeKernel(m_clKernelSphere, cl::NullRange, cl::NDRange(m_meshGeometry->faces.size()),
    //                                  cl::NullRange, nullptr);
    //   m_clQueue.finish();

    //   m_resultDistances.resize(m_meshGeometry->faces.size());
    //   m_clQueue.enqueueReadBuffer(m_clResultBuffer, CL_TRUE, 0, sizeof(float) * m_meshGeometry->faces.size(),
    //                               m_resultDistances.data());
    // }
    // catch (cl::Error err)
    // {
    //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), err.what() << ": " << CLUtil::getErrorString(err.err()));
    //   RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "(" << CLUtil::getErrorDescription(err.err()) << ")");
    // }

    for (int faceId = 0; faceId < m_meshGeometry->faces.size(); faceId++)
    {
      // if face is inside sphere, select it
      if (m_resultDistances[faceId] > 0)
      {
        if (m_faceSelectedArray.size() <= faceId)
        {
          m_faceSelectedArray.resize(faceId + 1);
        }
        m_faceSelectedArray[faceId] = selectMode;
      }
    }

    if (m_displayInitialized && m_visual)
    {
      std::vector<uint32_t> tmpFaceList;
      for (int faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
      {
        if (m_faceSelectedArray[faceId])
        {
          tmpFaceList.push_back(faceId);
        }
      }

      m_visual->setFacesInCluster(tmpFaceList);
    }
  }
}

boost::optional<std::pair<uint32_t, float>> ClusterLabelTool::getClosestIntersectedFaceParallel(
  Ogre::Ray& ray)
{
  m_rayData = { ray.getOrigin().x,    ray.getOrigin().y,    ray.getOrigin().z,
                ray.getDirection().x, ray.getDirection().y, ray.getDirection().z };

  // try
  // {
  //   m_clQueue.enqueueWriteBuffer(m_clRayBuffer, CL_TRUE, 0, sizeof(float) * 6, m_rayData.data());

  //   m_clQueue.enqueueNDRangeKernel(m_clKernelSingleRay, cl::NullRange, cl::NDRange(m_meshGeometry->faces.size()),
  //                                  cl::NullRange, nullptr);
  //   m_clQueue.finish();

  //   m_resultDistances.resize(m_meshGeometry->faces.size());
  //   m_clQueue.enqueueReadBuffer(m_clResultBuffer, CL_TRUE, 0, sizeof(float) * m_meshGeometry->faces.size(),
  //                               m_resultDistances.data());
  // }
  // catch (cl::Error err)
  // {
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), err.what() << ": " << CLUtil::getErrorString(err.err()));
  //   RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "(" << CLUtil::getErrorDescription(err.err()) << ")");
  // }

  uint32_t closestFaceId;
  bool faceFound = false;
  float minDist = std::numeric_limits<float>::max();

  for (uint32_t i = 0; i < m_meshGeometry->faces.size(); i++)
  {
    if (m_resultDistances[i] > 0 && m_resultDistances[i] < minDist)
    {
      closestFaceId = i;
      faceFound = true;
      minDist = m_resultDistances[i];
    }
  }

  if (faceFound)
  {
    return std::make_pair(closestFaceId, minDist);
  }
  else
  {
    return {};
  }
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
  if (event.leftDown() && event.control())
  {
    m_singleSelect = true;
    selectSphereFaces(event, true);
  }
  else if (event.leftUp() && m_singleSelect)
  {
    m_singleSelect = false;
    selectSphereFaces(event, true);
  }
  else if (event.rightDown() && event.control())
  {
    m_singleDeselect = true;
    selectSphereFaces(event, false);
  }
  else if (event.rightUp() && m_singleDeselect)
  {
    m_singleDeselect = false;
    selectSphereFaces(event, false);
  }
  else if (event.leftDown())
  {
    m_multipleSelect = true;
    selectionBoxStart(event);
  }
  else if (event.leftUp() && m_multipleSelect)
  {
    m_multipleSelect = false;
    selectMultipleFaces(event, true);
  }
  else if (event.rightDown())
  {
    m_multipleSelect = true;
    selectionBoxStart(event);
  }
  else if (event.rightUp() && m_multipleSelect)
  {
    m_multipleSelect = false;
    selectMultipleFaces(event, false);
  }
  else if (m_multipleSelect)
  {
    selectionBoxMove(event);
  }
  else if (m_singleSelect)
  {
    selectSphereFaces(event, true);
  }
  else if (m_singleDeselect)
  {
    selectSphereFaces(event, false);
  }

  return Render;
}

std::vector<uint32_t> ClusterLabelTool::getSelectedFaces()
{
  std::vector<uint32_t> faceList;

  for (int faceId = 0; faceId < m_faceSelectedArray.size(); faceId++)
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

}  // End namespace rviz_mesh_tools_plugins



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mesh_tools_plugins::ClusterLabelTool, rviz_common::Tool)
