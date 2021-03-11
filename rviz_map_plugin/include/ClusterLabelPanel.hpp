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
 *  ClusterLabelPanel.hpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#ifndef CLUSTER_LABEL_PANEL_HPP
#define CLUSTER_LABEL_PANEL_HPP

#include <Types.hpp>
#include <ros/ros.h>
#include <QObject>
#include <rviz/panel.h>
#include <rviz/tool_manager.h>
#include <ClusterLabelTool.hpp>
#include <mesh_msgs/MeshFaceCluster.h>

// Forward declarations
class QLineEdit;
class QPushButton;

namespace rviz
{
class Tool;
}

namespace rviz_map_plugin
{
/**
 * @class ClusterLabelPanel
 * @brief Panel for interacting with the label tool
 */
class ClusterLabelPanel : public rviz::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   * @param parent This panel's parent, if available
   */
  ClusterLabelPanel(QWidget* parent = 0);

  /**
   * @brief RViz callback on inizialize
   */
  void onInitialize();

  /**
   * @brief Load a configuration
   * @input config The configuration
   */
  virtual void load(const rviz::Config& config);

  /**
   * @brief Save a configuration
   * @input config The configuration
   */
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

  /**
   * @brief Set the name under which the current cluster will be saved
   * @param clusterName The new name
   */
  void setClusterName(const QString& clusterName);

  /**
   * @brief Updates the current cluster name
   */
  void updateClusterName();

  /**
   * @brief Publishes the current cluster
   */
  void publish();

  /**
   * @brief Resets the current face selection state
   */
  void resetFaces();

protected:
  /// Input for entering the cluster name
  QLineEdit* m_clusterNameEditor;
  /// Input for entering the output topic name
  QLineEdit* m_outputTopicEditor;

  /// Name of the cluster
  QString m_clusterName;

  /// Button for creating and publishing the cluster
  QPushButton* m_createClusterButton;
  /// Button for resetting the current faces in cluster
  QPushButton* m_resetFacesButton;

  /// Instance of the label tool from this package
  ClusterLabelTool* m_tool;

  /// Node handle
  ros::NodeHandle m_nodeHandle;
};

}  // end namespace rviz_map_plugin

#endif