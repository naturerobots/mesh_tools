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

// enable exceptions for OpenCL
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_ENABLE_EXCEPTIONS

#include <Types.hpp>

#include <CL/cl2.hpp>

#include <vector>
#include <map>
#include <memory>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include <QMessageBox>
#include <QApplication>
#include <QIcon>
#include <QObject>
#include <QWidget>
#include <QFrame>

#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/display.h>

#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>

#include <mesh_msgs/MeshFaceClusterStamped.h>

#ifndef Q_MOC_RUN
#include <rviz/mesh_loader.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreStringConverter.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneQuery.h>

#endif

namespace rviz
{
class RosTopicProperty;
class ColorProperty;
}  // namespace rviz

// OGRE stuff
namespace Ogre
{
// Forward declaration
class Vector3;
}  // namespace Ogre

namespace rviz_map_plugin
{
// Forward declarations
class ClusterLabelDisplay;
class ClusterLabelVisual;

/**
 * @class ClusterLabelTool
 * @brief Tool for selecting faces
 */
class ClusterLabelTool : public rviz::Tool
{
  Q_OBJECT
public:
  /**
   * @brief Constructor
   */
  ClusterLabelTool();

  /**
   * @brief Destructor
   */
  ~ClusterLabelTool();

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
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

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
   * @brief Adjust the sphere size for the brush tool
   * @param size The sphere size
   */
  void setSphereSize(float size);

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
  bool m_displayInitialized;
  ClusterLabelDisplay* m_display;
  std::shared_ptr<ClusterLabelVisual> m_visual;
  std::shared_ptr<Geometry> m_meshGeometry;
  float m_sphereSize = 1.0f;

  // Selection Box
  rviz::DisplayContext* m_displayContext;
  Ogre::SceneNode* m_sceneNode;
  Ogre::ManualObject* m_selectionBox;
  Ogre::MaterialPtr m_selectionBoxMaterial;
  Ogre::Vector2 m_selectionStart;
  Ogre::Vector2 m_selectionStop;
  bool m_multipleSelect = false;
  bool m_singleSelect = false;
  bool m_singleDeselect = false;

  std::vector<Ogre::Vector3> m_vertexPositions;

  void updateSelectionBox();
  void selectionBoxStart(rviz::ViewportMouseEvent& event);
  void selectionBoxMove(rviz::ViewportMouseEvent& event);
  void selectMultipleFaces(rviz::ViewportMouseEvent& event, bool selectMode);
  void selectFacesInBoxParallel(Ogre::PlaneBoundedVolume& volume, bool selectMode);
  void selectSingleFace(rviz::ViewportMouseEvent& event, bool selectMode);
  void selectSingleFaceParallel(Ogre::Ray& ray, bool selectMode);
  void selectSphereFaces(rviz::ViewportMouseEvent& event, bool selectMode);
  void selectSphereFacesParallel(Ogre::Ray& ray, bool selectMode);
  boost::optional<std::pair<uint32_t, float>> getClosestIntersectedFaceParallel(Ogre::Ray& ray);

  ros::Publisher m_labelPublisher;

  std::vector<float> m_vertexData;
  std::array<float, 6> m_rayData;
  std::array<float, 4> m_sphereData;
  std::array<float, 3> m_startNormalData;
  std::vector<float> m_boxData;
  std::vector<float> m_resultDistances;

  // OpenCL
  cl::Device m_clDevice;
  cl::Context m_clContext;
  cl::Program::Sources m_clProgramSources;
  cl::Program m_clProgram;
  cl::CommandQueue m_clQueue;
  cl::Buffer m_clVertexBuffer;
  cl::Buffer m_clResultBuffer;
  cl::Buffer m_clRayBuffer;
  cl::Buffer m_clSphereBuffer;
  cl::Buffer m_clBoxBuffer;
  cl::Buffer m_clStartNormalBuffer;
  cl::Kernel m_clKernelSingleRay;
  cl::Kernel m_clKernelSphere;
  cl::Kernel m_clKernelBox;
  cl::Kernel m_clKernelDirAndDist;
};
}  // end namespace rviz_map_plugin

#endif