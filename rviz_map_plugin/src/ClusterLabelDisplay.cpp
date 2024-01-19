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
 *  ClusterLabelDisplay.cpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#include <ClusterLabelDisplay.hpp>
#include <ClusterLabelVisual.hpp>
#include <ClusterLabelTool.hpp>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>

namespace rviz_map_plugin
{
Ogre::ColourValue getRainbowColor(float value)
{
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;

  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  float n = 1 - f;

  if (i <= 1)
    r = n, g = 0, b = 1;
  else if (i == 2)
    r = 0, g = n, b = 1;
  else if (i == 3)
    r = 0, g = 1, b = n;
  else if (i == 4)
    r = n, g = 1, b = 0;
  else if (i >= 5)
    r = 1, g = n, b = 0;

  return Ogre::ColourValue(r, g, b, 1.0f);
}

ClusterLabelDisplay::ClusterLabelDisplay()
{
  m_activeVisualProperty =
      new rviz::EnumProperty("Active label", "__NEW__", "Current active label. Can be edited with Cluster Label Tool",
                             this, SLOT(changeVisual()), this);
  m_alphaProperty = new rviz::FloatProperty("Transparency", 1.0f,
                                            "Transparency of the Labeled Cluster Visualization. 0.0 is fully "
                                            "transparent, 1.0 fully opaque",
                                            this, SLOT(updateColors()), this);
  m_alphaProperty->setMin(0.0f);
  m_alphaProperty->setMax(1.0f);

  m_colorsProperty = new rviz::Property("Colors", "", "colors", this, SLOT(updateColors()), this);
  m_colorsProperty->setReadOnly(true);
  m_sphereSizeProperty =
      new rviz::FloatProperty("Brush Size", 1.0f, "Brush Size", this, SLOT(updateSphereSize()), this);
  m_phantomVisualProperty = new rviz::BoolProperty("Show Phantom", false,
                                                   "Show a transparent silhouette of the whole mesh to help with "
                                                   "labeling",
                                                   this, SLOT(updatePhantomVisual()), this);

  setStatus(rviz::StatusProperty::Error, "Display", "Cant be used without Map3D plugin");
}

ClusterLabelDisplay::~ClusterLabelDisplay()
{
}

// =====================================================================================================================
// Public Q_SLOTS

std::shared_ptr<Geometry> ClusterLabelDisplay::getGeometry()
{
  if (!m_geometry)
  {
    ROS_ERROR("Label Display: Geometry requested, but none available!");
  }
  return m_geometry;
}

void ClusterLabelDisplay::setData(shared_ptr<Geometry> geometry, vector<Cluster> clusters)
{
  if (has_data)
  {
    ROS_WARN("Label Display: already has data, but setData() was called again!");
  }

  // Copy data
  m_geometry = geometry;
  m_clusterList = clusters;
  m_clusterList.insert(m_clusterList.begin(), Cluster("__NEW__", vector<uint32_t>()));

  // Set flag
  ROS_INFO("Label Display: received data");
  has_data = true;

  // Draw visuals
  if (isEnabled())
    updateMap();

  setStatus(rviz::StatusProperty::Ok, "Display", "");
}

// =====================================================================================================================
// Callbacks

void ClusterLabelDisplay::onInitialize()
{
  // Look for an existing label tool or create a new one
  initializeLabelTool();
}

void ClusterLabelDisplay::onEnable()
{
  updateMap();
}

void ClusterLabelDisplay::onDisable()
{
  m_visuals.clear();
  m_phantomVisual.reset();
  m_tool->resetVisual();
}

// =====================================================================================================================
// Callbacks triggered from UI events (mostly)

void ClusterLabelDisplay::changeVisual()
{
  if (m_activeVisualProperty->getStdString().empty())
  {
    ROS_ERROR("Label Display: Should change visual but no visual selected!");
    return;
  }

  ROS_INFO_STREAM("Label Display: Changed active visual to '" << m_activeVisualProperty->getStdString() << "'");

  m_activeVisualId = m_activeVisualProperty->getOptionInt();

  // Active visual has changed, notify label tool that it has to refresh its pointer on the active visual
  notifyLabelTool();
}

void ClusterLabelDisplay::updateMap()
{
  ROS_INFO("Label Display: Update");

  if (!has_data)
  {
    ROS_WARN("Label Display: No data available! Can't show map");
    return;
  }

  // Reset the visual of the label tool so that it can be deleted
  m_tool->resetVisual();

  // Now create the visuals for the loaded clusters
  createVisualsFromClusterList();

  // Fill options for dropdown menu containing the cluster names
  fillPropertyOptions();

  // Create a phantom visual if it is enabled
  updatePhantomVisual();

  // Notify label tool for changes. The label tool should now destroy its visual and get a new one from this obj
  notifyLabelTool();

  // Apply the default colors to the visuals
  updateColors();

  // Update the tool's assigned display (to this display)
  m_tool->setDisplay(this);

  // All good
  setStatus(rviz::StatusProperty::Ok, "Map", "");
}

void ClusterLabelDisplay::updateColors()
{
  for (int i = 0; i < m_colorProperties.size(); i++)
  {
    auto colorProp = m_colorProperties[i];
    m_visuals[i]->setColor(colorProp->getOgreColor(), m_alphaProperty->getFloat());
  }
}

void ClusterLabelDisplay::updateSphereSize()
{
  m_tool->setSphereSize(m_sphereSizeProperty->getFloat());
}

void ClusterLabelDisplay::updatePhantomVisual()
{
  if (!m_phantomVisualProperty->getBool())
  {
    m_phantomVisual.reset(nullptr);
  }
  else if (!m_phantomVisual)
  {
    createPhantomVisual();
  }
}

void ClusterLabelDisplay::fillPropertyOptions()
{
  // Clear options
  m_activeVisualProperty->clearOptions();
  m_colorsProperty->removeChildren();
  m_colorProperties.clear();

  for (int i = 0; i < m_clusterList.size(); i++)
  {
    // Add cluster labels to dropdown menu
    m_activeVisualProperty->addOption(QString::fromStdString(m_clusterList[i].name), i);

    // Add color options
    Ogre::ColourValue rainbowColor = getRainbowColor((((float)i + 1) / m_clusterList.size()));
    m_colorProperties.emplace_back(new rviz::ColorProperty(
        QString::fromStdString(m_clusterList[i].name),
        QColor(rainbowColor.r * 255, rainbowColor.g * 255, rainbowColor.b * 255),
        QString::fromStdString(m_clusterList[i].name), m_colorsProperty, SLOT(updateColors()), this));
  }
}

// =====================================================================================================================
// Visuals logic

void ClusterLabelDisplay::createVisualsFromClusterList()
{
  // Destroy all current visuals
  if (!m_visuals.empty())
  {
    m_visuals.clear();
  }

  // Create a visual for each entry in the cluster list
  float colorIndex = 0.0;  // index for coloring
  for (int i = 0; i < m_clusterList.size(); i++)
  {
    std::stringstream ss;
    ss << "ClusterLabelVisual_" << i;

    auto visual = std::make_shared<ClusterLabelVisual>(context_, ss.str(), m_geometry);
    ROS_DEBUG_STREAM("Label Display: Create visual for label '" << m_clusterList[i].name << "'");
    visual->setFacesInCluster(m_clusterList[i].faces);
    visual->setColor(getRainbowColor((++colorIndex / m_clusterList.size())), m_alphaProperty->getFloat());
    m_visuals.push_back(visual);
  }
}

void ClusterLabelDisplay::createPhantomVisual()
{
  m_phantomVisual.reset(new ClusterLabelVisual(context_, "ClusterLabelPhantomVisual", m_geometry));
  vector<uint32_t> allFacesVector;
  for (uint32_t i = 0; i < m_geometry->faces.size(); i++)
  {
    allFacesVector.push_back(i);
  }
  m_phantomVisual->setFacesInCluster(allFacesVector);
  m_phantomVisual->setColor(Ogre::ColourValue(0.2, 0.3, 0.2), 0.1);
}

// =====================================================================================================================
// Label tool

void ClusterLabelDisplay::initializeLabelTool()
{
  // Check if the cluster label tool is already opened
  rviz::ToolManager* toolManager = context_->getToolManager();
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
    m_tool = static_cast<ClusterLabelTool*>(context_->getToolManager()->addTool("rviz_map_plugin/ClusterLabel"));
  }
}

void ClusterLabelDisplay::notifyLabelTool()
{
  m_tool->setVisual(m_visuals[m_activeVisualId]);
}

void ClusterLabelDisplay::addLabel(std::string label, std::vector<uint32_t> faces)
{
  ROS_INFO_STREAM("Cluster Label Display: add label '" << label << "'");

  Q_EMIT signalAddLabel(Cluster(label, faces));
}

}  // End namespace rviz_map_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_map_plugin::ClusterLabelDisplay, rviz::Display)
