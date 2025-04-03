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

#include <rviz_mesh_tools_plugins/Types.hpp>
#include <rviz_mesh_tools_plugins/RvizFileProperty.hpp>

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
#include <QString>

#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/visualization_manager.hpp>
// #include <rviz_common/visualization_frame.hpp>
// #include <rviz_rendering/geometry.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/config.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/display_group.hpp>

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mesh_msgs/msg/mesh_geometry_stamped.hpp>
#include <mesh_msgs/msg/mesh_geometry.hpp>
#include <mesh_msgs/srv/get_geometry.hpp>
#include <mesh_msgs/srv/get_labeled_clusters.hpp>

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
#include <OgreColourValue.h>

#endif // Q_MOCK_RUN

#include <rviz_mesh_tools_plugins/ClusterLabelDisplay.hpp>
#include <rviz_mesh_tools_plugins/MeshDisplay.hpp>

namespace rviz
{
namespace properties
{
// Forward declaration
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class StringProperty;
class TfFrameProperty;
} // namespace properties
} // namespace rviz

namespace rviz_mesh_tools_plugins
{
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

/**
 * @class MapDisplay
 * @brief Master display for the Mesh- and Cluster- subdisplays. THis implementation uses HDF5 as it's data source
 */
class MapDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  using Base = rviz_common::Display;
  /**
   * @brief Constructor
   */
  MapDisplay();

  /**
   * @brief Destructor
   */
  ~MapDisplay();

  virtual void load(const rviz_common::Config& config) override;

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

  void updateMapFrame();

private:
  void enableClusterLabelDisplay();
  void disableClusterLabelDisplay();
  void enableMeshDisplay();
  void disableMeshDisplay();
  /**
   * @brief RViz callback on initialize
   */
  virtual void onInitialize() override;

  /**
   * @brief Periodically called from rviz
   */
  void update(float wall_dt, float ros_dt) override;

  /**
   * @brief RViz callback on enable
   */
  virtual void onEnable() override;

  /**
   * @brief RViz callback on disable
   */
  virtual void onDisable() override;

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
  rviz_common::properties::FileProperty* m_mapFilePath;
  std::string m_map_file_loaded;

  /// Set the TF Frame of the map
  rviz_common::properties::TfFrameProperty* m_mapTfFrame;

  /// Subdisplay: ClusterLabel (for showing the clusters)
  rviz_mesh_tools_plugins::ClusterLabelDisplay* m_clusterLabelDisplay;
  /// Subdisplay: MeshDisplay (for showing the mesh)
  rviz_mesh_tools_plugins::MeshDisplay* m_meshDisplay;

  /**
   * @brief Create a RViz display from it's unique class_id
   * @param class_id The class ID
   * @return Pointer to RViz display
   */
  rviz_common::Display* createDisplay(const QString& class_id);
};

}  // end namespace rviz_mesh_tools_plugins

#endif
