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
 *  ClusterLabelPanel.cpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#include <ClusterLabelPanel.hpp>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

#include <rviz/tool_manager.h>

namespace rviz_map_plugin
{
ClusterLabelPanel::ClusterLabelPanel(QWidget* parent) : rviz::Panel(parent)
{
  QHBoxLayout* clusterNameLayout = new QHBoxLayout();
  clusterNameLayout->addWidget(new QLabel("Cluster Name:"));
  m_clusterNameEditor = new QLineEdit();
  clusterNameLayout->addWidget(m_clusterNameEditor);

  m_createClusterButton = new QPushButton("Label Cluster");

  m_resetFacesButton = new QPushButton("Reset Faces");

  QVBoxLayout* layout = new QVBoxLayout();
  layout->addLayout(clusterNameLayout);
  layout->addWidget(m_createClusterButton);
  layout->addWidget(m_resetFacesButton);
  setLayout(layout);

  // Make signal/slot connections
  connect(m_clusterNameEditor, SIGNAL(editingFinished()), this, SLOT(updateClusterName()));

  connect(m_createClusterButton, SIGNAL(released()), this, SLOT(publish()));
  connect(m_resetFacesButton, SIGNAL(released()), this, SLOT(resetFaces()));
}

void ClusterLabelPanel::onInitialize()
{
  // Check if the cluster label tool is already opened
  rviz::ToolManager* toolManager = vis_manager_->getToolManager();
  QStringList toolClasses = toolManager->getToolClasses();
  bool foundTool = false;
  for (int i = 0; i < toolClasses.size(); i++)
  {
    if (toolClasses.at(i).contains("ClusterLabel"))
    {
      m_tool = static_cast<ClusterLabelTool*>(toolManager->getTool(i));
      foundTool = true;
      break;
    }
  }

  if (!foundTool)
  {
    m_tool = static_cast<ClusterLabelTool*>(vis_manager_->getToolManager()->addTool("rviz_map_plugin/ClusterLabel"));
  }
}

void ClusterLabelPanel::setClusterName(const QString& clusterName)
{
  m_clusterName = clusterName;
  Q_EMIT configChanged();

  // Gray out the create cluster button when the cluster name is empty
  m_createClusterButton->setEnabled(m_clusterName != "");
}

void ClusterLabelPanel::updateClusterName()
{
  setClusterName(m_clusterNameEditor->text());
}

void ClusterLabelPanel::publish()
{
  ROS_INFO("Label Panel: Publish");
  m_tool->publishLabel(m_clusterName.toStdString());
}

void ClusterLabelPanel::resetFaces()
{
  ROS_INFO("Label panel: Reset");
  m_tool->resetFaces();
}

void ClusterLabelPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("ClusterName", m_clusterName);
}

void ClusterLabelPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString clusterName;
  if (config.mapGetString("ClusterName", &clusterName))
    ;
  {
    m_clusterNameEditor->setText(clusterName);
    updateClusterName();
  }
}

}  // End namespace rviz_map_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_map_plugin::ClusterLabelPanel, rviz::Panel)
