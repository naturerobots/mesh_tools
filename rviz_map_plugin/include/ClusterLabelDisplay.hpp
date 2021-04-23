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
 *  ClusterLabelDisplay.hpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#ifndef CLUSTER_LABEL_DISPLAY_HPP
#define CLUSTER_LABEL_DISPLAY_HPP

#include <Types.hpp>

#include <vector>
#include <map>
#include <memory>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

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

#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/MeshGeometry.h>
#include <mesh_msgs/GetGeometry.h>
#include <mesh_msgs/GetLabeledClusters.h>

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
#include <OGRE/OgreColourValue.h>

#endif

namespace rviz
{
// Forward declaration
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class StringProperty;

}  // End namespace rviz

namespace rviz_map_plugin
{
using std::map;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

// Forward declaration
class ClusterLabelVisual;
class ClusterLabelTool;

/**
 * @class ClusterLabelDisplay
 * @brief Display class for the map plugin
 */
class ClusterLabelDisplay : public rviz::Display
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  ClusterLabelDisplay();

  /**
   * @brief Destructor
   */
  ~ClusterLabelDisplay();

  /**
   * @brief The tool will call this function and emit the signal below to the master display to
   *        create the label
   * @param label The label name
   * @param faces The list of face IDs
   */
  void addLabel(string label, vector<uint32_t> faces);

  /**
   * @brief RViz callback on enable
   */
  void onEnable();

  /**
   * @brief RViz callback on disable
   */
  void onDisable();

Q_SIGNALS:

  /**
   * @brief This signal is used for delegating new label data to the master display.
   * @param cluster The cluster
   */
  void signalAddLabel(Cluster cluster);

public Q_SLOTS:  // not sure wether any of those actually need to be q slots ...

  /**
   * @brief Refreshes the tool's current visual
   */
  void notifyLabelTool();

  /**
   * @brief Getter for the current geometry
   * @return The geometry
   */
  shared_ptr<Geometry> getGeometry();

  /**
   * @brief Setter for the geometry and cluster data
   * @param geometry The geometry
   * @param clusters The clusters
   */
  void setData(shared_ptr<Geometry> geometry, vector<Cluster> clusters);

private Q_SLOTS:

  /**
   * @brief Update the map, based on newly loaded data since the last update
   */
  void updateMap();

  /**
   * @brief Updates the colors, based on newly loaded data since the last update
   */
  void updateColors();

  /**
   * @brief Updates the sphere size for the brush tool
   */
  void updateSphereSize();

  /**
   * @brief Updates the phantom visual, based on newly loaded data since the last update
   */
  void updatePhantomVisual();

  /**
   * @brief Slot for changing the visual to the selected visual from the dropdown menu
   */
  void changeVisual();

private:
  /**
   * @brief RViz callback on initialize
   */
  void onInitialize();

  /**
   * @brief Programmatically create an instance of the label tool from this package
   */
  void initializeLabelTool();

  /**
   * @brief Create visuals for each cluster in the list
   */
  void createVisualsFromClusterList();

  /**
   * @brief Creates a phantom visual
   */
  void createPhantomVisual();

  /**
   * @brief Dynamically fills the dropdown menus of those properties
   */
  void fillPropertyOptions();

  /// Geometry
  shared_ptr<Geometry> m_geometry;

  /// Visuals
  vector<shared_ptr<ClusterLabelVisual>> m_visuals;

  /// ID of the current active visual
  uint32_t m_activeVisualId = 0;

  /// Additional visual to help with labeling without a TexturedMesh
  unique_ptr<ClusterLabelVisual> m_phantomVisual;

  /// Cluster data
  vector<Cluster> m_clusterList;

  /// Label tool
  ClusterLabelTool* m_tool;

  /// Property for the current active visual
  rviz::EnumProperty* m_activeVisualProperty;

  /// Property to set transparency
  rviz::FloatProperty* m_alphaProperty;

  /// Property for selecting colors (menu)
  rviz::Property* m_colorsProperty;

  /// Properties for selecting colors (menu-items)
  std::vector<rviz::ColorProperty*> m_colorProperties;

  /// Property to set the brushsize of the sphere brush of the label tool from this package
  rviz::FloatProperty* m_sphereSizeProperty;

  /// Property to hide or show a phantom visual
  rviz::BoolProperty* m_phantomVisualProperty;

  /// Index for the visuals
  int m_labelToolVisualIndex = 0;

  /// A variable that will be set to true, once the initial data has arrived
  bool has_data = false;
};

}  // end namespace rviz_map_plugin

#endif
