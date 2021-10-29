/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * created on: 30.04.2014
 *
 * conversions.cpp
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *         Henning Deeken <hdeeken@uos.de>,
 *         Marcel Mrozinski <mmrozins@uos.de>,
 *         Tristan Igelbrink <tigelbri@uos.de>
 *
 */

#include "mesh_msgs_conversions/conversions.h"
#include <omp.h>
#include <cmath>

namespace mesh_msgs_conversions
{

bool fromMeshBufferToMeshGeometryMessage(
    const lvr2::MeshBufferPtr& buffer,
    mesh_msgs::MeshGeometry& mesh_geometry
){
    size_t n_vertices = buffer->numVertices();
    size_t n_faces = buffer->numFaces();

    ROS_DEBUG_STREAM("Copy vertices from MeshBuffer to MeshGeometry.");

    // Copy vertices
    mesh_geometry.vertices.resize(n_vertices);
    auto buffer_vertices = buffer->getVertices();
    for (unsigned int i = 0; i < n_vertices; i++)
    {
        mesh_geometry.vertices[i].x = buffer_vertices[i * 3];
        mesh_geometry.vertices[i].y = buffer_vertices[i * 3 + 1];
        mesh_geometry.vertices[i].z = buffer_vertices[i * 3 + 2];
    }

    ROS_DEBUG_STREAM("Copy faces from MeshBuffer to MeshGeometry.");

    // Copy faces
    auto buffer_faces = buffer->getFaceIndices();
    mesh_geometry.faces.resize(n_faces);
    for (unsigned int i = 0; i < n_faces; i++)
    {
        mesh_geometry.faces[i].vertex_indices[0] = buffer_faces[i * 3];
        mesh_geometry.faces[i].vertex_indices[1] = buffer_faces[i * 3 + 1];
        mesh_geometry.faces[i].vertex_indices[2] = buffer_faces[i * 3 + 2];
    }

    // Copy vertex normals
    auto buffer_vertexnormals = buffer->getVertexNormals();
    if(buffer->hasVertexNormals())
    {
        ROS_DEBUG_STREAM("Copy normals from MeshBuffer to MeshGeometry.");

        mesh_geometry.vertex_normals.resize(n_vertices);
        for (unsigned int i = 0; i < n_vertices; i++) {
            mesh_geometry.vertex_normals[i].x = buffer_vertexnormals[i * 3];
            mesh_geometry.vertex_normals[i].y = buffer_vertexnormals[i * 3 + 1];
            mesh_geometry.vertex_normals[i].z = buffer_vertexnormals[i * 3 + 2];
        }
    }else{
        ROS_DEBUG_STREAM("No vertex normals given!");
    }

    ROS_DEBUG_STREAM("Successfully copied the MeshBuffer "
                         "geometry to the MeshGeometry message.");
    return true;
}

bool fromMeshBufferToMeshMessages(
    const lvr2::MeshBufferPtr& buffer,
    mesh_msgs::MeshGeometry& mesh_geometry,
    mesh_msgs::MeshMaterials& mesh_materials,
    mesh_msgs::MeshVertexColors& mesh_vertex_colors,
    boost::optional<std::vector<mesh_msgs::MeshTexture>&> texture_cache,
    std::string mesh_uuid
)
{
    size_t n_vertices = buffer->numVertices();
    size_t n_faces = buffer->numFaces();

    // copy vertices, faces and normals
    fromMeshBufferToMeshGeometryMessage(buffer, mesh_geometry);

    //size_t n_clusters = buffer->; TODO Clusters?
    // Copy clusters
    /*auto buffer_clusters = buffer->get;
    mesh_materials.clusters.resize(n_clusters);
    for (unsigned int i = 0; i < n_clusters; i++)
    {
        int n = buffer_clusters[i].size();
        mesh_materials.clusters[i].face_indices.resize(n);
        for (unsigned int j = 0; j < n; j++)
        {
            mesh_materials.clusters[i].face_indices[j] = buffer_clusters[i][j];
        }
    }
    buffer_clusters.clear();
    */

    size_t n_materials = buffer->getMaterials().size();
    size_t n_textures = buffer->getTextures().size();

    // Copy materials
    auto buffer_materials = buffer->getMaterials();
    mesh_materials.materials.resize(n_materials);
    for (unsigned int i = 0; i < n_materials; i++)
    {
        const lvr2::Material& m = buffer_materials[i];
        if (m.m_color)
        {
            mesh_materials.materials[i].color.r = m.m_color.get()[0]/255.0;
            mesh_materials.materials[i].color.g = m.m_color.get()[1]/255.0;
            mesh_materials.materials[i].color.b = m.m_color.get()[2]/255.0;
            mesh_materials.materials[i].color.a = 1.0;
        }
        else
        {
            mesh_materials.materials[i].color.r = 1.0;
            mesh_materials.materials[i].color.g = 1.0;
            mesh_materials.materials[i].color.b = 1.0;
            mesh_materials.materials[i].color.a = 1.0;
        }
        if (m.m_texture)
        {
            mesh_materials.materials[i].has_texture = true;
            mesh_materials.materials[i].texture_index = (int)m.m_texture.get().idx();
        }
        else
        {
            mesh_materials.materials[i].has_texture = false;
            mesh_materials.materials[i].texture_index = 0;
        }
    }
    buffer_materials.clear();

    // Copy cluster material indices TODO Cluster Materials?
    /*
    auto buffer_cluster_materials = buffer->getClusterMaterialIndices();
    mesh_materials.materials.resize(n_clusters);
    for (unsigned int i = 0; i < n_clusters; i++)
    {
        mesh_materials.cluster_materials[i] = buffer_cluster_materials[i];
    }
    buffer_cluster_materials.clear();
    */

    // Copy vertex tex coords
    auto buffer_texcoords = buffer->getTextureCoordinates();
    {
        mesh_materials.vertex_tex_coords.resize(n_vertices);

        for (unsigned int i = 0; i < n_vertices; i++)
        {
            mesh_materials.vertex_tex_coords[i].u = buffer_texcoords[i * 3];
            mesh_materials.vertex_tex_coords[i].v = buffer_texcoords[i * 3 + 1];
        }
    }

    // Copy vertex colors
    if (buffer->hasVertexColors())
    {
        size_t color_channels = 3;
        auto buffer_vertex_colors = buffer->getVertexColors(color_channels);
        mesh_vertex_colors.vertex_colors.resize(n_vertices);
        for (size_t i = 0; i < n_vertices; i++)
        {
            mesh_vertex_colors.vertex_colors[i].r = buffer_vertex_colors[i * 3 + 0]/255.0;
            mesh_vertex_colors.vertex_colors[i].g = buffer_vertex_colors[i * 3 + 1]/255.0;
            mesh_vertex_colors.vertex_colors[i].b = buffer_vertex_colors[i * 3 + 2]/255.0;
            mesh_vertex_colors.vertex_colors[i].a = 1.0;
        }
    }

    // If texture cache is available, cache textures in given vector
    if (texture_cache)
    {
        auto buffer_textures = buffer->getTextures();
        texture_cache.get().resize(n_textures);
        for (unsigned int i = 0; i < n_textures; i++)
        {
            sensor_msgs::Image image;
            sensor_msgs::fillImage(
                image,
                "rgb8",
                buffer_textures[i].m_height,
                buffer_textures[i].m_width,
                buffer_textures[i].m_width * 3, // step size
                buffer_textures[i].m_data
            );
            mesh_msgs::MeshTexture texture;
            texture.uuid = mesh_uuid;
            texture.texture_index = i;
            texture.image = image;
            texture_cache.get().at(i) = texture;
        }
        buffer_textures.clear();
    }

    return true;
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryConstPtr& mesh_geometry_ptr,
    lvr2::MeshBuffer& buffer)
{
    return fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, buffer);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryConstPtr& mesh_geometry_ptr,
    lvr2::MeshBufferPtr& buffer_ptr)
{
    if(!buffer_ptr) buffer_ptr = lvr2::MeshBufferPtr(new lvr2::MeshBuffer);
    return fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, *buffer_ptr);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryPtr& mesh_geometry_ptr,
    lvr2::MeshBufferPtr& buffer_ptr)
{
    if(!buffer_ptr) buffer_ptr = lvr2::MeshBufferPtr(new lvr2::MeshBuffer);
    return fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, *buffer_ptr);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryPtr& mesh_geometry_ptr,
    lvr2::MeshBuffer& buffer)
{
    return fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, buffer);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometry& mesh_geometry,
    lvr2::MeshBufferPtr& buffer_ptr)
{
    if(!buffer_ptr) buffer_ptr = lvr2::MeshBufferPtr(new lvr2::MeshBuffer);
    return fromMeshGeometryToMeshBuffer(mesh_geometry, *buffer_ptr);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometry& mesh_geometry,
    lvr2::MeshBuffer& buffer)
{

    const size_t numVertices = mesh_geometry.vertices.size();
    lvr2::floatArr vertices( new float[ numVertices * 3 ] );
    const auto& mg_vertices = mesh_geometry.vertices;
    for(size_t i; i<numVertices; i++)
    {
        vertices[ i * 3 + 0 ] = static_cast<float>(mg_vertices[i].x);
        vertices[ i * 3 + 1 ] = static_cast<float>(mg_vertices[i].y);
        vertices[ i * 3 + 2 ] = static_cast<float>(mg_vertices[i].z);
    }
    buffer.setVertices(vertices, numVertices);

    const size_t numFaces = mesh_geometry.faces.size();
    lvr2::indexArray faces( new unsigned int[ numVertices * 3 ] );
    const auto& mg_faces = mesh_geometry.faces;
    for(size_t i; i<numFaces; i++)
    {
        faces[ i * 3 + 0 ] = mg_faces[i].vertex_indices[0];
        faces[ i * 3 + 1 ] = mg_faces[i].vertex_indices[1];
        faces[ i * 3 + 2 ] = mg_faces[i].vertex_indices[2];
    }
    buffer.setFaceIndices(faces, numFaces);

    const size_t numNormals = mesh_geometry.vertex_normals.size();
    lvr2::floatArr normals( new float[ numNormals * 3 ] );
    const auto& mg_normals = mesh_geometry.vertex_normals;
    for(size_t i; i<numNormals; i++)
    {
        normals[ i * 3 + 0 ] = static_cast<float>(mg_normals[i].x);
        normals[ i * 3 + 1 ] = static_cast<float>(mg_normals[i].y);
        normals[ i * 3 + 2 ] = static_cast<float>(mg_normals[i].z);
    }
    buffer.setVertexNormals(normals);

    return true;
}

bool readMeshBuffer(lvr2::MeshBufferPtr& buffer_ptr, string path)
{
    lvr2::ModelFactory io_factory;
    lvr2::ModelPtr model = io_factory.readModel(path);

    if (!model)
    {
        return false;
    }
    else
    {
        buffer_ptr = model->m_mesh;
        return true;
    }
}

bool writeMeshBuffer(lvr2::MeshBufferPtr& buffer, string path)
{
    lvr2::ModelPtr model(new lvr2::Model(buffer));
    lvr2::ModelFactory::saveModel(model, path);
    return true;
}

/*
void removeDuplicates(lvr2::MeshBuffer& buffer)
{
    lvr2::floatArr old_vertexBuffer;
    lvr2::uintArr old_indexBuffer;
    std::vector<float> new_vertexBuffer;
    std::vector<unsigned int> new_indexBuffer;

    size_t old_numVertices, old_numIndices;
    size_t new_numVertices, new_numIndices;

    old_vertexBuffer = buffer.getVertexArray(old_numVertices);
    old_indexBuffer = buffer.getFaceArray(old_numIndices);

    std::map<lvr::Vertex<float>, unsigned int> vertexMap;
    size_t pos;
    int index;

    for (int i = 0; i < old_numIndices; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            index = old_indexBuffer[3 * i + j];

            lvr::Vertex<float> vertex =
                lvr::Vertex<float>(old_vertexBuffer[3 * index],
                                   old_vertexBuffer[3 * index + 1],
                                   old_vertexBuffer[3 * index + 2]);

            if (vertexMap.find(vertex) != vertexMap.end())
            {
                pos = vertexMap[vertex];
            }
            else
            {
                pos = new_vertexBuffer.size() / 3;
                new_vertexBuffer.push_back(vertex[0]);
                new_vertexBuffer.push_back(vertex[1]);
                new_vertexBuffer.push_back(vertex[2]);

                vertexMap.insert(pair<lvr::Vertex<float>, unsigned int>(vertex, pos));
            }

            new_indexBuffer.push_back(pos);
        }
    }
    buffer.setVertexArray(new_vertexBuffer);
    buffer.setFaceArray(new_indexBuffer);
}
*/
 
static inline bool hasCloudChannel(const sensor_msgs::PointCloud2& cloud, const std::string& field_name)
{
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size(); ++d)
        if (cloud.fields[d].name == field_name)
            return true;
    return false;
}

bool fromPointCloud2ToPointBuffer(const sensor_msgs::PointCloud2& cloud, lvr2::PointBuffer& buffer)
{
    size_t size = cloud.height * cloud.width;

    typedef sensor_msgs::PointCloud2ConstIterator<float> CloudIterFloat;
    typedef sensor_msgs::PointCloud2ConstIterator<uint8_t> CloudIterUInt8;

    std::list<int> filter_nan;

    // copy point data
    CloudIterFloat iter_x_filter(cloud, "x");
    CloudIterFloat iter_y_filter(cloud, "y");
    CloudIterFloat iter_z_filter(cloud, "z");

    // size without NaN values
    size = 0;
    for (int i = 0; iter_x_filter != iter_x_filter.end();
         ++iter_x_filter, ++iter_y_filter, ++iter_z_filter, i++)
    {
        if( !std::isnan(*iter_x_filter) &&
            !std::isnan(*iter_y_filter) &&
            !std::isnan(*iter_z_filter))
        {
            size++;
        }
        else
        {
            filter_nan.push_back(i);
        }
    }

    filter_nan.sort();

    lvr2::floatArr pointData(new float[size * 3]);

    // copy point data
    CloudIterFloat iter_x(cloud, "x");
    CloudIterFloat iter_y(cloud, "y");
    CloudIterFloat iter_z(cloud, "z");


    std::list<int> tmp_filter = filter_nan;
    int index, i;
    for (i = 0, index = 0;
         iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z,
         index++)
    {
        // skip NaN point values
        if (!tmp_filter.empty() && index == tmp_filter.front())
        {
            tmp_filter.pop_front();
            continue;
        }

        // copy point
        pointData[i] = *iter_x;
        pointData[i + 1] = *iter_y;
        pointData[i + 2] = *iter_z;

        i += 3;

    }
    buffer.setPointArray(pointData, size);

    // copy point normals if available
    bool normalsAvailable =
        hasCloudChannel(cloud, "normal_x")
        && hasCloudChannel(cloud, "normal_y")
        && hasCloudChannel(cloud, "normal_z");

    if (normalsAvailable)
    {
        CloudIterFloat iter_n_x(cloud, "normal_x");
        CloudIterFloat iter_n_y(cloud, "normal_y");
        CloudIterFloat iter_n_z(cloud, "normal_z");
        lvr2::floatArr normalsData(new float[size * 3]);
        tmp_filter = filter_nan;
        int index, i;
        for (i = 0, index = 0;
             iter_n_x != iter_n_x.end();
             ++iter_n_x, ++iter_n_y, ++iter_n_z,
                 index++)
        {
            // skip NaN point values
            if (!tmp_filter.empty() && index == tmp_filter.front())
            {
                tmp_filter.pop_front();
                continue;
            }

            // copy normal
            normalsData[i] = *iter_n_x;
            normalsData[i + 1] = *iter_n_y;
            normalsData[i + 2] = *iter_n_z;

            i += 3;
        }
        buffer.setNormalArray(normalsData, size);
    }

    // copy color data if available
    if (hasCloudChannel(cloud, "rgb"))
    {
        CloudIterUInt8 iter_rgb(cloud, "rgb");
        lvr2::ucharArr colorData(new uint8_t[size * 3]);
        tmp_filter = filter_nan;
        int index, i;
        for (i = 0, index = 0; iter_rgb != iter_rgb.end();
             ++iter_rgb, index++)
        {
            // skip NaN point values
            if (!tmp_filter.empty() && index == tmp_filter.front())
            {
                tmp_filter.pop_front();
                continue;
            }

            // copy color rgb
            colorData[i] = iter_rgb[0];
            colorData[i + 1] = iter_rgb[1];
            colorData[i + 2] = iter_rgb[2];

            i += 3;
        }
        buffer.setColorArray(colorData, size);
    }

    // copy intensity if available
    if (hasCloudChannel(cloud, "intensities"))
    {
        CloudIterFloat iter_int(cloud, "intensities");
        lvr2::floatArr intensityData(new float[size]);
        tmp_filter = filter_nan;
        int index, i;
        for (i = 0, index = 0; iter_int != iter_int.end();
             ++iter_int, index++)
        {
            // skip NaN point values
            if (!tmp_filter.empty() && index == tmp_filter.front())
            {
                tmp_filter.pop_front();
                continue;
            }

            // copy intensity
            intensityData[i] = *iter_int;
            i++;
        }
        buffer.addFloatChannel(intensityData, "intensity", size, 1);
    }
    return true;
}

bool fromMeshGeometryMessageToMeshBuffer(
    const mesh_msgs::MeshGeometry& mesh_geometry,
    const lvr2::MeshBufferPtr& buffer
)
{
    // copy vertices
    lvr2::floatArr vertices(new float[mesh_geometry.vertices.size() * 3]);
    int i = 0;
    for (auto vertex : mesh_geometry.vertices)
    {
        vertices[i] = static_cast<float>(vertex.x);
        vertices[i+1] = static_cast<float>(vertex.y);
        vertices[i+2] = static_cast<float>(vertex.z);
        i += 3;
    }
    buffer->setVertices(vertices, mesh_geometry.vertices.size());

    // copy faces
    lvr2::indexArray faces(new unsigned int[mesh_geometry.faces.size() * 3]);
    i = 0;
    for (auto face : mesh_geometry.faces)
    {
        faces[i] = face.vertex_indices[0];
        faces[i+1] = face.vertex_indices[1];
        faces[i+2] = face.vertex_indices[2];
        i += 3;
    }
    buffer->setFaceIndices(faces, mesh_geometry.faces.size() * 3);

    if(mesh_geometry.vertex_normals.size() == mesh_geometry.vertices.size())
    {
        // copy normals
        lvr2::floatArr normals(new float[mesh_geometry.vertex_normals.size() * 3]);
        i = 0;
        for (auto normal : mesh_geometry.vertex_normals)
        {
            normals[i] = static_cast<float>(normal.x);
            normals[i+1] = static_cast<float>(normal.y);
            normals[i+2] = static_cast<float>(normal.z);
            i += 3;
        }
        buffer->setVertexNormals(normals);
    }
    else
    {
        ROS_ERROR_STREAM("Number of normals (" << mesh_geometry.vertex_normals.size()
          << ") must be equal to number of vertices (" << mesh_geometry.vertices.size()
          << "), ignore normals!");
    }
    return true;
}

void PointBufferToPointCloud2(const lvr2::PointBufferPtr& buffer, std::string frame, sensor_msgs::PointCloud2Ptr& cloud) 
{ 
  // the offset will be updated by addPointField
  cloud->header.stamp = ros::Time::now();
  cloud->header.frame_id = frame;

  ros::Rate r(60);

  int type;
  std::map<std::string, lvr2::Channel<float> > floatChannels;
  type = buffer->getAllChannelsOfType<float>(floatChannels);
  std::map<std::string, lvr2::Channel<unsigned char> >  uCharChannels;
  int ucharType = buffer->getAllChannelsOfType<unsigned char>(uCharChannels);

  size_t size = 0;
  // xyz needs to be at the start.
  int offset = 4 * sizeof(float);
  // LVR mb needs a float64buffer especially for time information.
  for(auto channelPair: floatChannels)
  {
    // http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
    // For the padding reasons.
    if(channelPair.first == "points")
    {
      size = channelPair.second.numElements();
      int p_offset = 0;
      p_offset = addPointField(*cloud, "x", 1, sensor_msgs::PointField::FLOAT32, p_offset);
      p_offset = addPointField(*cloud, "y", 1, sensor_msgs::PointField::FLOAT32, p_offset);
      p_offset = addPointField(*cloud, "z", 1, sensor_msgs::PointField::FLOAT32, p_offset);
      p_offset += sizeof(float);
      cloud->point_step = offset;
      for(auto channelPair: uCharChannels)
      {
          if(channelPair.first == "colors")
          {

              offset = addPointField(*cloud, "rgb", channelPair.second.width(), sensor_msgs::PointField::FLOAT32, offset);
              cloud->point_step = offset;
          }
      }
    }
    else
    {
      // type in lvr2 starts at 0, in ros at 1
      offset = addPointField(*cloud, channelPair.first, channelPair.second.width(), type + 1, offset);
      //int padding = (sizeof(float) * 4) - ((channelPair.second.width() * sizeof(float)) % (4 * sizeof(float)));
      //offset += padding;
      cloud->point_step = offset;
    }
  }




//    // Is this useful?!
//    int padding = (sizeof(float) * 4) - ((channelPair.second.width() * sizeof(float)) % (4 * sizeof(float)));
//    offset += padding;
//    cloud->point_step = offset;
//  }

  std::map<std::string, lvr2::Channel<int> >  intChannels;
  type = buffer->getAllChannelsOfType<int>(intChannels);
  for(auto channelPair: intChannels)
  {
      // http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
      // For the padding reasons.
            // type in lvr2 starts at 0, in ros at 1
          offset = addPointField(*cloud, channelPair.first, channelPair.second.width(), type + 1, offset);
//          int padding = (sizeof(float) * 4) - ((channelPair.second.width() * sizeof(float)) % (4 * sizeof(float)));
//          offset += padding;
          cloud->point_step = offset;
  }

  // reserve size
  sensor_msgs::PointCloud2Modifier mod(*cloud);
  cloud->data.resize(size * cloud->point_step);
  
  cloud->height = 1;
  cloud->width = size;

 
  ROS_INFO("Starting conversion.");
  for(auto field: cloud->fields)
  {
    // Points is a special case...
    if(field.name == "x" || field.name == "y" || field.name == "z")
    {
      auto channel = floatChannels.at("points");
      //auto iter_x  = floatIters.at("x");
      //auto iter_y  = floatIters.at("y");
      //auto iter_z  = floatIters.at("z");
      #pragma omp parallel for
      for(size_t i = 0; i < size; ++i)
      {
        unsigned char* ptr = &(cloud->data[cloud->point_step * i]);
        *(reinterpret_cast<float*>(ptr))       = channel[i][0];
        *((reinterpret_cast<float*>(ptr)) + 1) = channel[i][1];
        *((reinterpret_cast<float*>(ptr)) + 2) = channel[i][2];
        //*iter_x = channel[i][0];
        //*iter_y = channel[i][1];
        //*iter_z = channel[i][2];
      }

    }
    else
    {
      if(field.datatype == sensor_msgs::PointField::FLOAT32)
      {
        std::string name = field.name;
        if(name == "rgb")
        {
            name = "colors";
            auto channel = uCharChannels.at(name);
            //auto iter = uCharIters.at(field.name);
#pragma omp parallel for
            for(size_t i = 0; i < size; ++i)
            {
                unsigned char* ptr = &(cloud->data[cloud->point_step * i]) + field.offset;
                *(ptr + 2) = channel[i][0];
                *(ptr + 1) = channel[i][1];
                *(ptr + 0) = channel[i][2];
                //for(size_t j = 0; j < field.count; ++j)
                //{
                //  //iter[j] = channel[i][j];
                //  *(ptr + j) = channel[i][j];
                //}
            }

            continue;
        }
        // ELSE
        auto channel = floatChannels.at(field.name);
        //auto iter = floatIters.at(field.name);
        #pragma omp parallel for
        for(size_t i = 0; i < size; ++i)
        {
          unsigned char* ptr = &(cloud->data[cloud->point_step * i]) + field.offset;
          for(size_t j = 0; j < field.count; ++j)
          {
            *((reinterpret_cast<float*>(ptr)) + j)  = channel[i][j];
            //iter[j]  = channel[i][j];
          }
        }
      }
      else if (field.datatype == sensor_msgs::PointField::UINT8)
      {
      }
    }
  }
  ROS_INFO("DONE");
}

void PointCloud2ToPointBuffer(const sensor_msgs::PointCloud2Ptr& cloud, lvr2::PointBufferPtr& buffer) 
{
   buffer = lvr2::PointBufferPtr(new lvr2::PointBuffer());

  for(auto field : cloud->fields)
  {
    if(field.datatype == sensor_msgs::PointField::FLOAT32)
    {
      if(field.name == "x" || field.name == "y" || field.name == "z")
      {
        if(!buffer->hasFloatChannel("points"))
        {
            buffer->addEmptyChannel<float>("points", cloud->width * cloud->height, 3);
        }
      }
      else
      {
        buffer->addEmptyChannel<float>(field.name, cloud->width * cloud->height, field.count);
      }
    
    }
  }

  for(auto field: cloud->fields)
  {
    // Points is a special case...
    if(field.name == "x" || field.name == "y" || field.name == "z")
    {
      auto channel = buffer->getChannel<float>("points");
      if(channel)
      {
        std::cout << "already init" << std::endl;
        continue;
      }
      
      lvr2::floatArr points(new float[cloud->width * cloud->height * 3]);
      buffer->setPointArray(points, cloud->width * cloud->height);
      channel = buffer->getChannel<float>("points");

      #pragma omp parallel for
      for(size_t i = 0; i < (cloud->width * cloud->height); ++i)
      {
        unsigned char* ptr = &(cloud->data[cloud->point_step * i]);
        (*channel)[i][0] = *(reinterpret_cast<float*>(ptr));
        (*channel)[i][1] = *((reinterpret_cast<float*>(ptr)) + 1);
        (*channel)[i][2] = *((reinterpret_cast<float*>(ptr)) + 2);
      }

    }
    else
    {
      if(field.datatype == sensor_msgs::PointField::FLOAT32)
      {
        auto channel = buffer->getChannel<float>(field.name);

        if(!channel)
        {
          ROS_INFO("Channel %s missing", field.name.c_str());
          continue;
        }

        #pragma omp parallel for
        for(size_t i = 0; i < (cloud->width * cloud->height); ++i)
        {
          unsigned char* ptr = &(cloud->data[cloud->point_step * i]) + field.offset;
          for(size_t j = 0; j < field.count; ++j)
          {
            (*channel)[i][j] = *((reinterpret_cast<float*>(ptr)) + j);
          }
        }
      }
      else if (field.datatype == sensor_msgs::PointField::UINT8)
      {
        auto channel = buffer->getChannel<unsigned char>(field.name);

        if(!channel)
        {
          ROS_INFO("Channel %s missing", field.name.c_str());
          continue;
        }

        #pragma omp parallel for
        for(size_t i = 0; i < (cloud->width * cloud->height); ++i)
        {
          unsigned char* ptr = &(cloud->data[cloud->point_step * i]) + field.offset;
          for(size_t j = 0; j < field.count; ++j)
          {
            (*channel)[i][j] = *(ptr + j);
          }
        }
      }
    }
  }
}

} // end namespace
