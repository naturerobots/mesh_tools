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
 *  MapDisplay.cpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#include <MapDisplay.hpp>
#include <ClusterLabelVisual.hpp>
#include <ClusterLabelTool.hpp>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>

#include <rviz/failed_display.h>
#include <rviz/display_factory.h>

namespace rviz_map_plugin
{
MapDisplay::MapDisplay()
{
  m_mapFilePath = new rviz::FileProperty("Map file path", "/path/to/map.h5", "Absolute path of the map file", this,
                                         SLOT(updateMap()));
}

MapDisplay::~MapDisplay()
{
}

// =====================================================================================================================
// Public Q_SLOTS

std::shared_ptr<Geometry> MapDisplay::getGeometry()
{
  if (!m_geometry)
  {
    ROS_ERROR("Map Display: Geometry requested, but none available!");
  }
  return m_geometry;
}

// =====================================================================================================================
// Callbacks

rviz::Display* MapDisplay::createDisplay(const QString& class_id)
{
  rviz::DisplayFactory* factory = context_->getDisplayFactory();
  QString error;
  rviz::Display* disp = factory->make(class_id, &error);
  if (!disp)
  {
    return new rviz::FailedDisplay(class_id, error);
  }
  return disp;
}

void MapDisplay::onInitialize()
{
  Display* display = createDisplay("rviz_map_plugin/ClusterLabel");

  m_clusterLabelDisplay = static_cast<ClusterLabelDisplay*>(display);
  m_clusterLabelDisplay->setName("ClusterLabel");
  m_clusterLabelDisplay->setModel(model_);
  m_clusterLabelDisplay->setParent(this);
  addChild(m_clusterLabelDisplay);
  m_clusterLabelDisplay->initialize(context_);

  Display* meshDisplay = createDisplay("rviz_map_plugin/Mesh");

  m_meshDisplay = static_cast<MeshDisplay*>(meshDisplay);
  addChild(m_meshDisplay);
  m_meshDisplay->setName("Mesh");
  m_meshDisplay->setModel(model_);
  m_meshDisplay->setParent(this);
  m_meshDisplay->initialize(context_);
  m_meshDisplay->ignoreIncomingMessages();

  // Make signal/slot connections
  connect(m_clusterLabelDisplay, SIGNAL(signalAddLabel(Cluster)), this, SLOT(saveLabel(Cluster)));

  updateMap();
}

void MapDisplay::onEnable()
{
  m_clusterLabelDisplay->onEnable();
  m_meshDisplay->onEnable();
}

void MapDisplay::onDisable()
{
  m_clusterLabelDisplay->onDisable();
  m_meshDisplay->onDisable();
}

// =====================================================================================================================
// Callbacks triggered from UI events (mostly)

void MapDisplay::updateMap()
{
  ROS_INFO("Map Display: Update");

  // Load geometry and clusters
  bool successful = loadData();
  if (!successful)
    return;

  // Update sub-plugins
  m_meshDisplay->setGeometry(m_geometry);
  m_meshDisplay->setVertexColors(m_colors);
  m_meshDisplay->setVertexNormals(m_normals);
  m_meshDisplay->clearVertexCosts();
  for (const auto& vertexCosts : m_costs)
  {
      std::vector<float> costs = vertexCosts.second;
      m_meshDisplay->addVertexCosts(vertexCosts.first, costs);
  }
  m_meshDisplay->setMaterials(m_materials, m_texCoords);
  // m_meshDisplay->setTexCoords(m_texCoords);
  for (uint32_t i = 0; i < m_textures.size(); i++)
  {
    m_meshDisplay->addTexture(m_textures[i], i);
  }
  m_clusterLabelDisplay->setData(m_geometry, m_clusterList);

  // All good
  setStatus(rviz::StatusProperty::Ok, "Map", "");
}

// =====================================================================================================================
// Data loading

bool MapDisplay::loadData()
{
  // Read map file path
  std::string mapFile = m_mapFilePath->getFilename();
  if (mapFile.empty())
  {
    ROS_WARN_STREAM("Map Display: No map file path specified!");
    setStatus(rviz::StatusProperty::Warn, "Map", "No map file path specified!");
    return false;
  }
  if (!boost::filesystem::exists(mapFile))
  {
    ROS_WARN_STREAM("Map Display: Specified map file does not exist!");
    setStatus(rviz::StatusProperty::Warn, "Map", "Specified map file does not exist!");
    return false;
  }
  if (boost::filesystem::extension(mapFile).compare(".h5") != 0)
  {
    ROS_WARN_STREAM("Map Display: Specified map file is not a .h5 file!");
    setStatus(rviz::StatusProperty::Warn, "Map", "Specified map file is not a .h5 file!");
    return false;
  }
  ROS_INFO_STREAM("Map Display: Loading data for map '" << mapFile << "'");

  try
  {
    // Open file IO
    hdf5_map_io::HDF5MapIO map_io(mapFile);

    ROS_INFO("Map Display: Load geometry");

    // Read geometry
    m_geometry = std::make_shared<Geometry>(Geometry(map_io.getVertices(), map_io.getFaceIds()));

    ROS_INFO("Map Display: Load textures");

    // Read textures
    vector<hdf5_map_io::MapImage> textures = map_io.getTextures();
    m_textures.resize(textures.size());
    for (size_t i = 0; i < textures.size(); i++)
    {
      // Find out the texture index because textures are not stored in ascending order
      int textureIndex = std::stoi(textures[i].name);

      // Copy metadata
      m_textures[textureIndex].width = textures[i].width;
      m_textures[textureIndex].height = textures[i].height;
      m_textures[textureIndex].channels = textures[i].channels;
      m_textures[textureIndex].data = textures[i].data;
      m_textures[textureIndex].pixelFormat = "rgb8";
    }

    ROS_INFO("Map Display: Load materials");

    // Read materials
    vector<hdf5_map_io::MapMaterial> materials = map_io.getMaterials();
    vector<uint32_t> faceToMaterialIndexArray = map_io.getMaterialFaceIndices();
    m_materials.resize(materials.size());
    for (size_t i = 0; i < materials.size(); i++)
    {
      // Copy material color
      m_materials[i].color.r = materials[i].r / 255.0f;
      m_materials[i].color.g = materials[i].g / 255.0f;
      m_materials[i].color.b = materials[i].b / 255.0f;
      m_materials[i].color.a = 1.0f;

      // Look for texture index
      if (materials[i].textureIndex == -1)
      {
        // texture index -1: no texture
        m_materials[i].textureIndex = boost::none;
      }
      else
      {
        m_materials[i].textureIndex = materials[i].textureIndex;
      }

      m_materials[i].faceIndices.clear();
    }

    // Copy face indices
    for (size_t k = 0; k < faceToMaterialIndexArray.size(); k++)
    {
      m_materials[faceToMaterialIndexArray[k]].faceIndices.push_back(k);
    }

    ROS_INFO("Map Display: Load vertex colors");

    // Read vertex colors
    vector<uint8_t> colors = map_io.getVertexColors();
    m_colors.clear();
    m_colors.reserve(colors.size() / 3);
    for (size_t i = 0; i < colors.size(); i += 3)
    {
      // convert from 0-255 (uint8) to 0.0-1.0 (float)
      m_colors.push_back(Color(colors[i + 0] / 255.0f, colors[i + 1] / 255.0f, colors[i + 2] / 255.0f, 1.0));
    }

    ROS_INFO("Map Display: Load vertex normals");

    // Read vertex normals
    vector<float> normals = map_io.getVertexNormals();
    m_normals.clear();
    m_normals.reserve(normals.size() / 3);
    for (size_t i = 0; i < normals.size(); i += 3)
    {
      m_normals.push_back(Normal(normals[i + 0], normals[i + 1], normals[i + 2]));
    }

    ROS_INFO("Map Display: Load texture coordinates");

    // Read tex cords
    vector<float> texCoords = map_io.getVertexTextureCoords();
    m_texCoords.clear();
    m_texCoords.reserve(texCoords.size() / 3);
    for (size_t i = 0; i < texCoords.size(); i += 3)
    {
      m_texCoords.push_back(TexCoords(texCoords[i], texCoords[i + 1]));
    }

    ROS_INFO("Map Display: Load clusters");

    // Read labels
    m_clusterList.clear();
    // m_clusterList.push_back(Cluster("__NEW__", vector<uint32_t>()));
    for (auto labelGroup : map_io.getLabelGroups())
    {
      for (auto labelObj : map_io.getAllLabelsOfGroup(labelGroup))
      {
        auto faceIds = map_io.getFaceIdsOfLabel(labelGroup, labelObj);

        std::stringstream ss;
        ss << labelGroup << "_" << labelObj;
        std::string label = ss.str();

        m_clusterList.push_back(Cluster(label, faceIds));
      }
    }

    m_costs.clear();
    for (std::string costlayer : map_io.getCostLayers())
    {
        try
        {
            m_costs[costlayer] = map_io.getVertexCosts(costlayer);
        }
        catch (const hf::DataSpaceException& e)
        {
            ROS_WARN_STREAM("Could not load channel " << costlayer << " as a costlayer!");
        }
    }
  }
  catch (...)
  {
    ROS_ERROR_STREAM("An unexpected error occurred while using Pluto Map IO");
    setStatus(rviz::StatusProperty::Error, "IO", "An unexpected error occurred while using Pluto Map IO");
    return false;
  }

  setStatus(rviz::StatusProperty::Ok, "IO", "");

  ROS_INFO("Map Display: Successfully loaded map.");

  return true;
}

// =====================================================================================================================
// Label

void MapDisplay::saveLabel(Cluster cluster)
{
  std::string label = cluster.name;
  std::vector<uint32_t> faces = cluster.faces;

  ROS_INFO_STREAM("Map Display: add label '" << label << "'");

  try
  {
    // Split label into class and instance (tree_1 => class "tree" & instance "1")
    std::vector<std::string> results;
    boost::split(results, label, [](char c) { return c == '_'; });
    if (results.size() != 2)
    {
      ROS_ERROR_STREAM("Map Display: Illegal label name '" << label << "'");
      setStatus(rviz::StatusProperty::Error, "Label", "Illegal label name!");
      return;
    }

    // Open IO
    hdf5_map_io::HDF5MapIO map_io(m_mapFilePath->getFilename());

    // Add label with faces list
    map_io.addOrUpdateLabel(results[0], results[1], faces);

    // Add to cluster list
    m_clusterList.push_back(Cluster(label, faces));

    setStatus(rviz::StatusProperty::Ok, "Label", "Successfully saved label");
    ROS_INFO_STREAM("Map Display: Successfully added label to map.");

    // update the map to show the new label
    updateMap();
  }
  catch (...)
  {
    setStatus(rviz::StatusProperty::Error, "Label", "Error while saving label");
  }
}

}  // End namespace rviz_map_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_map_plugin::MapDisplay, rviz::Display)
