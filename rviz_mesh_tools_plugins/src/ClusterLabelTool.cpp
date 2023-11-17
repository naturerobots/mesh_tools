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

#include <rviz_rendering/render_window.hpp>

#include <OgreColourValue.h>
#include <OgreCamera.h>
#include <OgreTechnique.h>
#include <OgreViewport.h>

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
{
  shortcut_key_ = 'l';
}

ClusterLabelTool::~ClusterLabelTool()
{
  m_selectedFaces.clear();
  context_->getSceneManager()->destroyManualObject(m_selectionBox->getName());
  context_->getSceneManager()->destroyManualObject(m_selectionBoxMaterial->getName());
  context_->getSceneManager()->destroySceneNode(m_sceneNode);
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


  // // try-catch block to check for OpenCL errors
  // try
  // {
  //   // Initialize OpenCL
  //   RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Get platforms");
  //   vector<cl::Platform> platforms;
  //   cl::Platform::get(&platforms);
  //   for (auto const& platform : platforms)
  //   {
  //     RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found platform: %s", platform.getInfo<CL_PLATFORM_NAME>().c_str());
  //     RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "platform version: %s", platform.getInfo<CL_PLATFORM_VERSION>().c_str());
  //   }
  //   RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), " ");

  //   vector<cl::Device> consideredDevices;
  //   for (auto const& platform : platforms)
  //   {
  //     RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Get devices of %s: ", platform.getInfo<CL_PLATFORM_NAME>().c_str());
  //     cl_context_properties properties[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)(platform)(), 0 };
  //     m_clContext = cl::Context(CL_DEVICE_TYPE_ALL, properties);
  //     vector<cl::Device> devices = m_clContext.getInfo<CL_CONTEXT_DEVICES>();
  //     for (auto const& device : devices)
  //     {
  //       RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found device: %s", device.getInfo<CL_DEVICE_NAME>().c_str());
  //       RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Device work units: %d", device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>());
  //       RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Device work group size: %lu", device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>());

  //       std::string device_info = device.getInfo<CL_DEVICE_VERSION>();
  //       // getVersion extracts the version number with major in the upper 16 bits and minor in the lower 16 bits

  //       unsigned int version = cl::detail::getVersion(std::vector<char>(device_info.begin(), device_info.end()));

  //       // shift 16 to the right to get the number in the upper 16 bits
  //       cl_uint majorVersion = version >> 16;
  //       // use bitwise AND to extract the number in the lower 16 bits
  //       cl_uint minorVersion = version & 0x0000FFFF;

  //       RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Found a device with OpenCL version: %u.%u", majorVersion, minorVersion);

  //       // find all devices that support at least OpenCL version 1.2
  //       if (majorVersion >= 1 && minorVersion >= 2)
  //         ;
  //       {
  //         consideredDevices.push_back(device);
  //       }
  //     }
  //   }
  //   RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), " ");

  //   cl::Platform platform;
  //   // Preferably choose the first compatible device of type GPU
  //   bool deviceFound = false;
  //   for (auto const& device : consideredDevices)
  //   {
  //     if (device.getInfo<CL_DEVICE_TYPE>() == CL_DEVICE_TYPE_GPU)
  //     {
  //       m_clDevice = device;
  //       platform = device.getInfo<CL_DEVICE_PLATFORM>();
  //       deviceFound = true;
  //       break;
  //     }
  //   }
  //   if (!deviceFound && consideredDevices.size() > 0)
  //   {
  //     // If no device of type GPU was found, choose the first compatible device
  //     m_clDevice = consideredDevices[0];
  //     platform = m_clDevice.getInfo<CL_DEVICE_PLATFORM>();
  //     deviceFound = true;
  //   }
  //   if (!deviceFound)
  //   {
  //     // Panic if no compatible device was found
  //     RCLCPP_ERROR(rclcpp::get_logger("rviz_mesh_tools_plugins"), "No device with compatible OpenCL version found (minimum 2.0)");
  //     // auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  //     rclcpp::shutdown();
  //   }

  //   cl_context_properties properties[] = { CL_CONTEXT_PLATFORM, (cl_context_properties)(platform)(), 0 };
  //   m_clContext = cl::Context(CL_DEVICE_TYPE_ALL, properties);

  //   RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Using device %s of platform %s", m_clDevice.getInfo<CL_DEVICE_NAME>().c_str(),
  //            platform.getInfo<CL_PLATFORM_NAME>().c_str());
  //   RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), " ");

  //   // Read kernel file 
  //   // may throw ament_index_cpp::PackageNotFoundError exception
  //   std::string package_share_directory = ament_index_cpp::get_package_share_directory("my_package_name");

  //   ifstream in(package_share_directory + CL_RAY_CAST_KERNEL_FILE);
  //   std::string cast_rays_kernel(static_cast<stringstream const&>(stringstream() << in.rdbuf()).str());


  //   RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Got kernel: %s%s", package_share_directory.c_str(), CL_RAY_CAST_KERNEL_FILE);

  //   m_clProgramSources = cl::Program::Sources(1, { cast_rays_kernel.c_str(), cast_rays_kernel.length() });

  //   m_clProgram = cl::Program(m_clContext, m_clProgramSources);
  //   try
  //   {
  //     m_clProgram.build({ m_clDevice });
  //     RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Successfully built program.");
  //   }
  //   catch (cl::Error& err)
  //   {
      
  //     RCLCPP_ERROR(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Error building: %s", m_clProgram.getBuildInfo<CL_PROGRAM_BUILD_LOG>(m_clDevice).c_str());
  //     // auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  //     rclcpp::shutdown();
  //     // ros::shutdown();
  //     exit(1);
  //   }

  //   // Create queue to which we will push commands for the device.
  //   m_clQueue = cl::CommandQueue(m_clContext, m_clDevice, 0);
  // }
  // catch (cl::Error err)
  // {
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), err.what() << ": " << CLUtil::getErrorString(err.err()));
  //   RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "(" << CLUtil::getErrorDescription(err.err()) << ")");
  //   // auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  //   rclcpp::shutdown();
  //   // ros::requestShutdown();
  //   exit(1);
  // }
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

  // try-catch block to check for OpenCL errors
  // try
  // {
  //   m_clVertexBuffer = cl::Buffer(m_clContext, CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY | CL_MEM_COPY_HOST_PTR,
  //                                 sizeof(float) * m_vertexData.size(), m_vertexData.data());

  //   m_clResultBuffer = cl::Buffer(m_clContext, CL_MEM_WRITE_ONLY | CL_MEM_HOST_READ_ONLY,
  //                                 sizeof(float) * m_meshGeometry->faces.size());

  //   m_clRayBuffer = cl::Buffer(m_clContext, CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, sizeof(float) * 6);

  //   m_clSphereBuffer = cl::Buffer(m_clContext, CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, sizeof(float) * 4);

  //   m_clBoxBuffer = cl::Buffer(m_clContext, CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, sizeof(float) * 4 * 6);

  //   m_clStartNormalBuffer = cl::Buffer(m_clContext, CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, sizeof(float) * 3);

  //   m_clKernelSingleRay = cl::Kernel(m_clProgram, "cast_rays");
  //   m_clKernelSphere = cl::Kernel(m_clProgram, "cast_sphere");
  //   m_clKernelBox = cl::Kernel(m_clProgram, "cast_box");

  //   m_clKernelSingleRay.setArg(0, m_clVertexBuffer);
  //   m_clKernelSingleRay.setArg(1, m_clRayBuffer);
  //   m_clKernelSingleRay.setArg(2, m_clResultBuffer);

  //   m_clKernelSphere.setArg(0, m_clVertexBuffer);
  //   m_clKernelSphere.setArg(1, m_clSphereBuffer);
  //   m_clKernelSphere.setArg(2, m_clResultBuffer);
  //   m_clKernelSphere.setArg(3, m_sphereSize);

  //   m_clKernelBox.setArg(0, m_clVertexBuffer);
  //   m_clKernelBox.setArg(1, m_clBoxBuffer);
  //   m_clKernelBox.setArg(2, m_clResultBuffer);
  // }
  // catch (cl::Error err)
  // {
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), err.what() << ": " << CLUtil::getErrorString(err.err()));
  //   RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "(" << CLUtil::getErrorDescription(err.err()) << ")");
  //   // auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  //   rclcpp::shutdown();
  //   // ros::shutdown();
  //   exit(1);
  // }
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

  int left = m_bb_x1;
  int right = m_bb_x2;
  int top = m_bb_y1;
  int bottom = m_bb_y2;

  size_t goalSection;
  size_t goalIndex;

  if (left > right)
  {
    std::swap(left, right);
  }

  if (top > bottom)
  {
    std::swap(top, bottom);
  }

  const int BOX_SIZE_TOLERANCE = 1;
  if ((right - left) * (bottom - top) < BOX_SIZE_TOLERANCE)
  {
    // single face
    selectSingleFace(event, selectMode);
  } else {
    // std::cout << "PICK!" << std::endl;
    rviz_common::interaction::M_Picked pick_results;

    auto manager = context_->getSelectionManager();

    manager->pick(
      m_evt_panel->getRenderWindow(),
      m_bb_x1, m_bb_y1, 
      m_bb_x2, m_bb_y2, 
      pick_results);

    if(!pick_results.empty())
    {
      // FOUND SOMETHING!
      // std::cout << "FOUND SOMETHING! " << pick_results.size() << std::endl;

      for(auto elem : pick_results)
      {
        rviz_common::interaction::CollObjectHandle key = elem.first;
        rviz_common::interaction::Picked value = elem.second;
        
        // std::cout << key << std::endl;
        // std::cout << "Extra handles: " << value.extra_handles.size() << std::endl;

      }

      // TODO: continue implementing this
    } else {
      // std::cout << "NOTHING to pick :(" << std::endl;
    }
  }

  // Ogre::PlaneBoundedVolume volume =
  //     event.panel->getViewController()->getCamera()->getCameraToViewportBoxVolume(left, top, right, bottom, true);

  // selectFacesInBoxParallel(volume, selectMode);

  // auto manager = context_->getSelectionManager();

  // virtual void SelectionManager::pick(
    // rviz_rendering::RenderWindow * window,
    // int x1,
    // int y1,
    // int x2,
    // int y2,
    // M_Picked & results) = 0;


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
  // TODO: Segfault here, when using RViz config
  m_visual.reset();
}

}  // End namespace rviz_mesh_tools_plugins



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mesh_tools_plugins::ClusterLabelTool, rviz_common::Tool)