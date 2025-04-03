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

#include <rviz_mesh_tools_plugins/MapDisplay.hpp>
#include <rviz_mesh_tools_plugins/ClusterLabelVisual.hpp>
#include <rviz_mesh_tools_plugins/ClusterLabelTool.hpp>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>

// #include <rviz_common/failed_display.hpp>
// #include <rviz_common/display_factory.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display_context.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <OgreImage.h>
#include <OgreDataStream.h>
#include <fstream>

#include <lvr2/io/deprecated/hdf5/MeshIO.hpp>
#include <lvr2/io/deprecated/hdf5/PointCloudIO.hpp>

#include <unordered_set>


using HDF5MeshIO = lvr2::Hdf5IO<>::AddFeatures<
  lvr2::hdf5features::MeshIO,
  lvr2::hdf5features::PointCloudIO>;

namespace rviz_mesh_tools_plugins
{

template<typename T>
std::vector<T> copy_1d_channel_to_vector(const lvr2::Channel<T>& lvr_channel)
{
    if(lvr_channel.width() != 1)
    {
        throw std::runtime_error("channel is not 1d!");
    }
    std::vector<T> ret(lvr_channel.numElements());
    for(size_t i=0; i<lvr_channel.numElements(); i++)
    {
        ret[i] = lvr_channel[i][0];
    }
    return ret;
}

MapDisplay::MapDisplay()
: m_mapTfFrame(nullptr)
, m_clusterLabelDisplay(nullptr)
, m_meshDisplay(nullptr)
{
  m_mapFilePath = new rviz_common::properties::FileProperty("Map file path", "/path/to/map.h5", "Absolute path of the map file", this,
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
    RCLCPP_ERROR(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Geometry requested, but none available!");
  }
  return m_geometry;
}

// =====================================================================================================================
// Callbacks

rviz_common::Display* MapDisplay::createDisplay(const QString& class_id)
{
  rviz_common::Display* disp = context_->getRootDisplayGroup()->createDisplay(class_id);

  if (!disp)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rviz_mesh_tools_plugins"), "IM SEARCHING FOR rviz_common::FailedDisplay");
  }
  return disp;
}

void MapDisplay::enableClusterLabelDisplay()
{
  if(!m_clusterLabelDisplay)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "createDisplay: rviz_mesh_tools_plugins/ClusterLabel");
    Display* display = createDisplay("rviz_mesh_tools_plugins/ClusterLabel");
    if (m_clusterLabelDisplay = dynamic_cast<ClusterLabelDisplay*>(display); m_clusterLabelDisplay != nullptr)
    {
      m_clusterLabelDisplay->setName("ClusterLabel");
      m_clusterLabelDisplay->setModel(model_);
      m_clusterLabelDisplay->setParent(this);
      addChild(m_clusterLabelDisplay);
      m_clusterLabelDisplay->initialize(context_);

      // Make signal/slot connections
      connect(m_clusterLabelDisplay, SIGNAL(signalAddLabel(Cluster)), this, SLOT(saveLabel(Cluster)));
      RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "createDisplay: rviz_mesh_tools_plugins/ClusterLabel. CREATED");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("rviz_mesh_tools_plugins"), "createDisplay: rviz_mesh_tools_plugins/ClusterLabel. NOT FOUND");
    }
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "rviz_mesh_tools_plugins/ClusterLabel. ALREADY EXISTING");
    m_clusterLabelDisplay->onEnable();
    m_clusterLabelDisplay->show();
  }
}

void MapDisplay::disableClusterLabelDisplay()
{
  if(m_clusterLabelDisplay)
  {
    m_clusterLabelDisplay->onDisable();
    m_clusterLabelDisplay->hide();
  }
}

// TODO: name it "constructMeshDisplay" or "initMeshDisplay"
void MapDisplay::enableMeshDisplay()
{
  if(!m_meshDisplay)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "createDisplay: rviz_mesh_tools_plugins/Mesh");
    Display* meshDisplay = createDisplay("rviz_mesh_tools_plugins/Mesh");
    if(m_meshDisplay = dynamic_cast<MeshDisplay*>(meshDisplay); m_meshDisplay != nullptr)
    {
      addChild(m_meshDisplay);
      m_meshDisplay->setName("Mesh");
      m_meshDisplay->setModel(model_);
      m_meshDisplay->setParent(this);
      m_meshDisplay->initialize(context_);
      m_meshDisplay->ignoreIncomingMessages();
      RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "createDisplay: rviz_mesh_tools_plugins/Mesh. CREATED");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("rviz_mesh_tools_plugins"), "createDisplay: rviz_mesh_tools_plugins/Mesh. NOT FOUND");
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "rviz_mesh_tools_plugins/Mesh. ENABLE");
  m_meshDisplay->setEnabled(true); // doesnt trigger onEnable
  m_meshDisplay->onEnable();
  m_meshDisplay->show();
}

void MapDisplay::disableMeshDisplay()
{
  if(m_meshDisplay)
  {
    m_meshDisplay->setEnabled(false);
    m_meshDisplay->hide();
  }
}

void MapDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();

  std::string name = this->getName().toStdString();
  RCLCPP_DEBUG(rclcpp::get_logger("rviz_mesh_tools_plugins"), "createDisplay: rviz_mesh_tools_plugins/ClusterLabel");

  // make ROS parameters visible
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  std::stringstream ss;
  ss << "rviz_mesh_tools_plugins." << name;
  if(!node->has_parameter(ss.str()))
  {
    node->declare_parameter(ss.str(), "");
  }
}

void MapDisplay::update(float wall_dt, float ros_dt)
{
  // Pass update down to the subplugins
  if (m_meshDisplay)
  {
    m_meshDisplay->update(wall_dt, ros_dt);
  }
  if (m_clusterLabelDisplay)
  {
    m_clusterLabelDisplay->update(wall_dt, ros_dt);
  }
}

void MapDisplay::onEnable()
{
  if(m_clusterLabelDisplay)
  {
    if(m_clusterLabelDisplay->isEnabled())
    {
      m_clusterLabelDisplay->onEnable();
    }
  }
  
  if(m_meshDisplay)
  {
    if(m_meshDisplay->isEnabled())
    {
      m_meshDisplay->onEnable();
    }
  }
}

void MapDisplay::onDisable()
{
  if(m_clusterLabelDisplay)
  {
    m_clusterLabelDisplay->onDisable();
  }
  if(m_meshDisplay)
  {
    m_meshDisplay->onDisable();
  }
}

// =====================================================================================================================
// Callbacks triggered from UI events (mostly)

void MapDisplay::load(const rviz_common::Config& config)
{
  std::string name = this->getName().toStdString();
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), name << ": LOAD CONFIG...");

  rviz_common::Config config2;
  config2.copy(config);

  { // Override with ros params
    std::stringstream ss;
    ss << "rviz_mesh_tools_plugins" << "." << name;

    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

    std::string mesh_file;
    if(node->get_parameter(ss.str(), mesh_file))
    {
      if(mesh_file != "")
      {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Loading " << mesh_file << " that was set with parameter " << ss.str());
        config2.mapSetValue(m_mapFilePath->getName(), QString::fromStdString(mesh_file) );
      }
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), name << ": COULDN'T FIND MESH TO LOAD: " << mesh_file);
    }
  }

  Base::load(config2);

  // parts of the childs are not loaded automatically?. Do it manually instead. TODO: figure out why
  if(m_meshDisplay)
  {
    auto mesh_config = config2.mapGetChild(m_meshDisplay->getName());
    if(mesh_config.isValid())
    {
      m_meshDisplay->load(mesh_config);
    }
  }

  if(m_clusterLabelDisplay)
  {
    auto cluster_label_config = config2.mapGetChild(m_clusterLabelDisplay->getName());
    if(cluster_label_config.isValid())
    {
      m_clusterLabelDisplay->load(cluster_label_config);
    }
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), name << ": LOAD CONFIG done.");
}

void MapDisplay::updateMap()
{
  std::string name = this->getName().toStdString();

  // Load geometry and clusters
  bool successful = loadData();
  if (!successful)
  {
    return;
  }

  if (nullptr == m_mapTfFrame)
  {
    m_mapTfFrame = new rviz_common::properties::TfFrameProperty(
      "Frame",
      "map",
      "The TF frame of the loaded map.",
      this,
      nullptr,
      true,
      SLOT(updateMapFrame())
    );
    m_mapTfFrame->setFrameManager(context_->getFrameManager());
  }

  if(m_meshDisplay)
  {
    // Update sub-plugins
    m_meshDisplay->setGeometry(m_geometry);
    m_meshDisplay->setVertexColors(m_colors);
    m_meshDisplay->setVertexNormals(m_normals);
    m_meshDisplay->clearVertexCosts();
    m_meshDisplay->setMapFrame(m_mapTfFrame->getFrameStd());
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
  }

  if(m_clusterLabelDisplay)
  {
    m_clusterLabelDisplay->setData(m_geometry, m_clusterList);
  }

  // All good
  setStatus(rviz_common::properties::StatusProperty::Ok, "Map", "");

  m_map_file_loaded = m_mapFilePath->getFilename();
}

void MapDisplay::updateMapFrame()
{
  if (m_meshDisplay && m_mapTfFrame)
  {
    m_meshDisplay->setMapFrame(m_mapTfFrame->getStdString());
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("rviz_mesh_tools_plugins"),
    "Map frame set to %s", m_mapTfFrame->getFrameStd().c_str()
  );
}

// =====================================================================================================================
// Data loading

bool MapDisplay::loadData()
{
  std::string name = this->getName().toStdString();

  if(m_mapFilePath->getFilename() == m_map_file_loaded)
  {
    std::cout << name << "! Tried to load same map twice. Skipping and keeping old data" << std::endl;
    return true;
  }

  // Read map file path
  std::string mapFile = m_mapFilePath->getFilename();
  if (mapFile.empty())
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: No map file path specified!");
    setStatus(rviz_common::properties::StatusProperty::Warn, "Map", "No map file path specified!");
    return false;
  }
  if (!boost::filesystem::exists(mapFile))
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Specified map file does not exist!");
    setStatus(rviz_common::properties::StatusProperty::Warn, "Map", "Specified map file does not exist!");
    return false;
  }
  
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Loading data for map '" << mapFile << "'");

  try
  {
    if (boost::filesystem::extension(mapFile).compare(".h5") == 0)
    {
      enableClusterLabelDisplay(); // enable label writing to hdf5
      enableMeshDisplay();

      RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Load HDF5 map");

      auto hdf5_mesh_io = std::make_shared<HDF5MeshIO>();

      // TODO: dynamic
      const std::string mesh_part = "mesh";

      hdf5_mesh_io->open(mapFile);
      auto mesh_buffer = hdf5_mesh_io->MeshIO::load(mesh_part);

      // the mesh buffer is a map from a string to a Channel
      // example:
      // "vertices" -> Channel<float>
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Loaded: " << *mesh_buffer);

      // For "mesh_navigation_tutorials/maps/floor_is_lava.h5":
      // Loaded: [ VariantChannelMap ]
      // vertices: type: float, size: [8100,3]
      // vertex_normals: type: float, size: [8100,3]
      // face_indices: type: unsigned int, size: [15960,3]

      RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Load geometry");

      // Read geometry
      m_geometry = std::make_shared<Geometry>();
      { // fill
        lvr2::Channel<float> lvr_vertices = *mesh_buffer->getChannel<float>("vertices");
        
        m_geometry->vertices.resize(lvr_vertices.numElements());
        for(size_t i=0; i<lvr_vertices.numElements(); i++)
        {
          m_geometry->vertices[i].x = lvr_vertices[i][0];
          m_geometry->vertices[i].y = lvr_vertices[i][1];
          m_geometry->vertices[i].z = lvr_vertices[i][2];
        }

        lvr2::Channel<unsigned int> lvr_face_ids = *mesh_buffer->getChannel<unsigned int>("face_indices");

        m_geometry->faces.resize(lvr_face_ids.numElements());
        for(size_t i=0; i<lvr_face_ids.numElements(); i++)
        {
          m_geometry->faces[i].vertexIndices[0] = lvr_face_ids[i][0];
          m_geometry->faces[i].vertexIndices[1] = lvr_face_ids[i][1];
          m_geometry->faces[i].vertexIndices[2] = lvr_face_ids[i][2];
        }
      }

      RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Load vertex colors");

      // Read vertex colors
      m_colors.clear();
      if(auto lvr2_vertex_colors_opt = mesh_buffer->getChannel<unsigned char>("vertex_colors"))
      {
        lvr2::Channel<unsigned char> lvr_vertex_colors = *lvr2_vertex_colors_opt;
        m_colors.resize(lvr_vertex_colors.numElements());
        const bool has_alpha = lvr_vertex_colors.width() > 3;
        for (size_t i = 0; i < lvr_vertex_colors.numElements(); i ++)
        {
          // convert from 0-255 (uint8) to 0.0-1.0 (float)
          m_colors[i].r = lvr_vertex_colors[i][0] / 255.0f;
          m_colors[i].g = lvr_vertex_colors[i][1] / 255.0f;
          m_colors[i].b = lvr_vertex_colors[i][2] / 255.0f;
          if(has_alpha)
          {
            m_colors[i].a = lvr_vertex_colors[i][3] / 255.0f;
          } else {
            m_colors[i].a = 1.0;
          }
        }
      }

      RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Load vertex normals");

      // Read vertex normals
      m_normals.clear();
      if(auto lvr2_vertex_normals_opt = mesh_buffer->getChannel<float>("vertex_normals"))
      {
        lvr2::Channel<float> lvr2_vertex_normals = *lvr2_vertex_normals_opt;
        m_normals.reserve(lvr2_vertex_normals.numElements());
        for (size_t i = 0; i < lvr2_vertex_normals.numElements(); i++)
        {
          m_normals.push_back(Normal(
            lvr2_vertex_normals[i][0],
            lvr2_vertex_normals[i][1],
            lvr2_vertex_normals[i][2]));
        }
      }

      RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Load vertex costs...");

      auto vertex_attributes = hdf5_mesh_io->PointCloudIO::load(mesh_part + "/vertex_attributes");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Vertex Costs: " << *vertex_attributes);
      
      // For "mesh_navigation_tutorials/maps/floor_is_lava.h5":
      // Vertex Attributes: [ VariantChannelMap ]
      // vertex_normals: type: float, size: [8100,3]
      // roughness: type: float, size: [8100,1]
      // inflation: type: float, size: [8100,1]
      // height_diff: type: float, size: [8100,1]
      // border: type: float, size: [8100,1]

      m_costs.clear();
      for(auto elem : *vertex_attributes)
      { 
        // this is checking if a channel is a vertex cost channel
        // I suggest for future changes to somehow mark a channel as "vertex_cost" instead
        if(elem.second.is_type<float>()
          && elem.second.numElements() == m_geometry->vertices.size()
          && elem.second.width() == 1)
        {
          // vertex channel of type float
          RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Loading '" << elem.first << "' as vertex cost layer");
          m_costs[elem.first] = copy_1d_channel_to_vector(elem.second.extract<float>());
        }
      }
    }
    else 
    {
      disableClusterLabelDisplay(); // we cannot write labels to standard formats
      enableMeshDisplay();
      RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "LOADING WITH ASSIMP");

      // PLY, OBJ, DAE? -> ASSIMP
      // The following lines are a simple way to import the mesh geometry
      // of commonly used mesh file formats.
      //
      // TODOs:
      // 1. scene graphs will not be imported properly.
      //    Someone has to do some transformations according to the 
      //    node graph in the assimp structures. Or optionally (even better): 
      //    create tf-transformations for every element of the scene graph#
      // 2. HDF5 is used to store more information such as label etc.
      //    So we possibly need to transform the geometry from PLY, OBJ, DAE to H5 first??
      // 
      Assimp::Importer io;
      io.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
      
      // with aiProcess_PreTransformVertices assimp transforms the whole scene graph
      // into one mesh
      // - if you want to use TF for spawning meshes, the loading has to be done manually
      const aiScene* ascene = io.ReadFile(mapFile, aiProcess_PreTransformVertices | aiProcess_Triangulate | aiProcess_SortByPType);
      if (!ascene) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Error while loading map: " << io.GetErrorString());
        setStatus(rviz_common::properties::StatusProperty::Error, "IO", io.GetErrorString());
        return false;
      }

      m_geometry = std::make_shared<Geometry>();
      m_normals.clear();
      m_colors.clear();
      m_texCoords.clear();
      m_textures.clear();
      m_materials.clear();
      m_materials.resize(ascene->mNumMaterials);
      int numTextures = 0;

      // load all meshes into one geometry
      for (unsigned int meshIdx = 0; meshIdx < ascene->mNumMeshes; meshIdx++)
      {
        const aiMesh* amesh = ascene->mMeshes[meshIdx];

        // skip non-triangle meshes
        if (amesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE)
        {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Mesh " << meshIdx << " is not a triangle mesh! Skipping...");
          continue;
        }

        // save old vector sizes
        unsigned int numVertices = m_geometry->vertices.size();
        unsigned int numFaces = m_geometry->faces.size();

        m_geometry->vertices.resize(numVertices + amesh->mNumVertices);
        m_geometry->faces.resize(numFaces + amesh->mNumFaces);

      RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "- Vertices, Faces: " << amesh->mNumVertices << ", " << amesh->mNumFaces);

      for (unsigned int i = 0; i < amesh->mNumVertices; i++)
      {
        m_geometry->vertices[numVertices + i].x = amesh->mVertices[i].x;
        m_geometry->vertices[numVertices + i].y = amesh->mVertices[i].y;
        m_geometry->vertices[numVertices + i].z = amesh->mVertices[i].z;
      }

      for (unsigned int i = 0; i < amesh->mNumFaces; i++)
      {
        m_geometry->faces[numFaces + i].vertexIndices[0] = numVertices + amesh->mFaces[i].mIndices[0];
        m_geometry->faces[numFaces + i].vertexIndices[1] = numVertices + amesh->mFaces[i].mIndices[1];
        m_geometry->faces[numFaces + i].vertexIndices[2] = numVertices + amesh->mFaces[i].mIndices[2];
      }

      m_normals.resize(numVertices + amesh->mNumVertices, Normal(0.0, 0.0, 0.0));
      if(amesh->HasNormals())
      {
        for(unsigned int i=0; i<amesh->mNumVertices; i++)
        {
          m_normals[numVertices + i].x = amesh->mNormals[i].x;
          m_normals[numVertices + i].y = amesh->mNormals[i].y;
          m_normals[numVertices + i].z = amesh->mNormals[i].z;
        }
      }

      // assimp supports more color channels but not this plugin
      // can we support this too?
      m_colors.resize(numVertices + amesh->mNumVertices, Color(1.0, 1.0, 1.0, 1.0));
      int color_channel_id = 0;
      if(amesh->HasVertexColors(color_channel_id))
      {
        for(unsigned int i=0; i<amesh->mNumVertices; i++)
        {
          m_colors[numVertices + i].r = amesh->mColors[color_channel_id][i].r;
          m_colors[numVertices + i].g = amesh->mColors[color_channel_id][i].g;
          m_colors[numVertices + i].b = amesh->mColors[color_channel_id][i].b;
          m_colors[numVertices + i].a = amesh->mColors[color_channel_id][i].a;
        }
      }

        // store texture coordinates
        m_texCoords.resize(numVertices + amesh->mNumVertices, TexCoords(0.0, 0.0));
        if (amesh->HasTextureCoords(0))
        {
          for (unsigned int i = 0; i < amesh->mNumVertices; i++)
          {
            m_texCoords[numVertices + i].u = amesh->mTextureCoords[0][i].x;
            m_texCoords[numVertices + i].v = amesh->mTextureCoords[0][i].y;
          }
        }

        if (ascene->HasMaterials())
        {
          // load material
          aiMaterial* material = ascene->mMaterials[amesh->mMaterialIndex];
          
          aiColor3D color;
          material->Get(AI_MATKEY_COLOR_DIFFUSE, color);

          // store material and adjacent faces
          m_materials[amesh->mMaterialIndex].color.r = color.r;
          m_materials[amesh->mMaterialIndex].color.g = color.g;
          m_materials[amesh->mMaterialIndex].color.b = color.b;
          m_materials[amesh->mMaterialIndex].color.a = 1.0f;

          m_materials[amesh->mMaterialIndex].faceIndices.resize(amesh->mNumFaces);
          std::iota(m_materials[amesh->mMaterialIndex].faceIndices.begin(), m_materials[amesh->mMaterialIndex].faceIndices.end(), numFaces);

          m_materials[amesh->mMaterialIndex].textureIndex = boost::none;

          // load textures from file
          aiString textureFile;
          if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0)
          {
            material->GetTexture(aiTextureType_DIFFUSE, 0, &textureFile);
            
            // get current file path
            boost::filesystem::path mapFilePath(mapFile);
            boost::filesystem::path texturePath = mapFilePath.parent_path() / textureFile.C_Str();

            // If the texture image doesn't exist then try the next most likely path
            if (!boost::filesystem::exists(texturePath))
            {
              texturePath = mapFilePath.parent_path() / "../materials/textures" / textureFile.C_Str();
              if (!boost::filesystem::exists(texturePath))
              {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Texture: " << texturePath.c_str() << " could not be found!");
                continue;
              }
            }

            std::ifstream ifs(texturePath.c_str(), std::ios::binary|std::ios::in);
            if (ifs.is_open())
            {
              Ogre::FileStreamDataStream* file_stream = new Ogre::FileStreamDataStream(texturePath.c_str(), &ifs, false);
              Ogre::DataStreamPtr data_stream(file_stream);
              Ogre::Image img;
              auto textureExt = texturePath.extension().string();
              img.load(data_stream, textureExt.substr(1, textureExt.size() - 1));

              m_textures.push_back(Texture());
              m_textures[numTextures].width = img.getWidth();
              m_textures[numTextures].height = img.getHeight();
              m_textures[numTextures].channels = 3;
              m_textures[numTextures].data.resize(img.getWidth() * img.getHeight() * m_textures[numTextures].channels);
              m_textures[numTextures].pixelFormat = "rgb8";

              // scale image in order to get the right pixel format
              Ogre::PixelBox pb(img.getWidth(), img.getHeight(), img.getDepth(), Ogre::PF_BYTE_RGB, img.getData());
              Ogre::Image::scale(img.getPixelBox(), pb);
              uchar* pixelData = static_cast<uchar*>(pb.data);
              unsigned int indexData = 0;
              for (unsigned int y = 0; y < pb.getHeight(); y++)
              {
                for (unsigned int x = 0; x < pb.getWidth(); x++)
                {
                  unsigned int indexPData = y * pb.rowPitch * 3 + x * 3;
                  m_textures[numTextures].data[indexData++] = pixelData[indexPData + 0];
                  m_textures[numTextures].data[indexData++] = pixelData[indexPData + 1];
                  m_textures[numTextures].data[indexData++] = pixelData[indexPData + 2];
                }
              }

              m_materials[amesh->mMaterialIndex].textureIndex = numTextures++;
            }
            else
            {
              RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Texture: " << texturePath.c_str() << " could not be opened!");
            }

            ifs.close();
          }
        }
      }

      // delete texCoords if there are no textures
      if (m_textures.empty())
      {
        m_texCoords.clear();
      }

      m_costs.clear();
    }
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "An unexpected error occurred while loading map.");
    setStatus(rviz_common::properties::StatusProperty::Error, "IO", "An unexpected error occurred while loading map.");
    return false;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "IO", "");

  RCLCPP_INFO(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Successfully loaded map.");

  return true;
}

// =====================================================================================================================
// Label

void MapDisplay::saveLabel(Cluster cluster)
{
  std::string label = cluster.name;
  std::vector<uint32_t> faces = cluster.faces;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: add label '" << label << "'");

  try
  {
    // Split label into class and instance (tree_1 => class "tree" & instance "1")
    std::vector<std::string> results;
    boost::split(results, label, [](char c) { return c == '_'; });
    if (results.size() != 2)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Illegal label name '" << label << "'");
      setStatus(rviz_common::properties::StatusProperty::Error, "Label", "Illegal label name!");
      return;
    }

    // is not working anyway
    // // Open IO
    // hdf5_map_io::HDF5MapIO map_io(m_mapFilePath->getFilename());

    // // Add label with faces list
    // map_io.addOrUpdateLabel(results[0], results[1], faces);

    // Add to cluster list
    m_clusterList.push_back(Cluster(label, faces));

    setStatus(rviz_common::properties::StatusProperty::Ok, "Label", "Successfully saved label");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rviz_mesh_tools_plugins"), "Map Display: Successfully added label to map.");

    // update the map to show the new label
    updateMap();
  }
  catch (...)
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Label", "Error while saving label");
  }
}

}  // End namespace rviz_mesh_tools_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_mesh_tools_plugins::MapDisplay, rviz_common::Display)
