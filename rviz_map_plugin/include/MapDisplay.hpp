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
 *  MapDisplay.hpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#ifndef MAP_DISPLAY_HPP
#define MAP_DISPLAY_HPP

#include <Types.hpp>
#include "RvizFileProperty.hpp"

#include <vector>
#include <memory>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <math.h>
#include <algorithm>
#include <map>

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

#include <hdf5_map_io/hdf5_map_io.h>

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

#include <ClusterLabelDisplay.hpp>
#include <MeshDisplay.hpp>

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
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

/**
 * @class MapDisplay
 * @brief Master display for the Mesh- and Cluster- subdisplays. THis implementation uses HDF5 as it's data source
 */
class MapDisplay : public rviz::Display
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  MapDisplay();

  /**
   * @brief Destructor
   */
  ~MapDisplay();

public Q_SLOTS:

  /**
   * @brief Saves a label to HDF5
   * @param cluster The cluster to be saved
   */
  void saveLabel(Cluster cluster);

  /**
   * @brief Get the geometry
   * @return The geometry
   */
  shared_ptr<Geometry> getGeometry();

private Q_SLOTS:

  /**
   * @brief Update the map, based on the current data state
   */
  void updateMap();

private:
  /**
   * @brief RViz callback on initialize
   */
  void onInitialize();

  /**
   * @brief RViz callback on enable
   */
  void onEnable();

  /**
   * @brief RViz callback on disable
   */
  void onDisable();

  /**
   * @brief Read all data from the HDF5 file and save it in the member variables
   * @return true, if successful
   */
  bool loadData();

  // TODO: make more efficient - currently everything is stored in the MapDisplay, the MeshDisplay and the MeshVisual
  /// Geometry
  shared_ptr<Geometry> m_geometry;
  /// Materials
  vector<Material> m_materials;
  /// Textures
  vector<Texture> m_textures;
  /// Colors
  vector<Color> m_colors;
  /// Vertex normals
  vector<Normal> m_normals;
  /// Texture coordinates
  vector<TexCoords> m_texCoords;
  /// Clusters
  vector<Cluster> m_clusterList;

  std::map<std::string, std::vector<float>> m_costs;

  /// Path to map file
  rviz::FileProperty* m_mapFilePath;

  /// Subdisplay: ClusterLabel (for showing the clusters)
  rviz_map_plugin::ClusterLabelDisplay* m_clusterLabelDisplay;
  /// Subdisplay: MeshDisplay (for showing the mesh)
  rviz_map_plugin::MeshDisplay* m_meshDisplay;

  /**
   * @brief Create a RViz display from it's unique class_id
   * @param class_id The class ID
   * @return Pointer to RViz display
   */
  rviz::Display* createDisplay(const QString& class_id);
};

}  // end namespace rviz_map_plugin

#endif
