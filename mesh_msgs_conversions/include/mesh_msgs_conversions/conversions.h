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
 * conversions.h
 *
 * created on: 30.04.2014
 *
 * Author: Henning Deeken <hdeeken@uos.de>,
 *         Sebastian Pütz <spuetz@uos.de>,
 *         Marcel Mrozinski <mmrozins@uos.de>,
 *         Tristan Igelbrink <tigelbri@uos.de>
 *
 */

#ifndef MESH_MSGS_CONVERSIONS_H_
#define MESH_MSGS_CONVERSIONS_H_

#include <map>

#include "rclcpp/rclcpp.hpp"

#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/types/PointBuffer.hpp>
#include <lvr2/types/MeshBuffer.hpp>
#include <lvr2/geometry/BaseMesh.hpp>
#include <lvr2/attrmaps/AttrMaps.hpp>

#include <lvr2/types/Model.hpp>
#include <lvr2/io/DataStruct.hpp>
#include <lvr2/io/ModelFactory.hpp>

#include <lvr2/texture/Texture.hpp>
#include <lvr2/geometry/BaseMesh.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>

#include "std_msgs/msg/string.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/fill_image.hpp"

#include "mesh_msgs/msg/mesh_face_cluster.hpp"
#include "mesh_msgs/msg/mesh_material.hpp"
#include "mesh_msgs/msg/mesh_triangle_indices.hpp"

#include "mesh_msgs/msg/mesh_geometry.hpp"
#include "mesh_msgs/msg/mesh_geometry_stamped.hpp"
#include "mesh_msgs/msg/mesh_materials_stamped.hpp"
#include "mesh_msgs/msg/mesh_vertex_colors.hpp"
#include "mesh_msgs/msg/mesh_vertex_colors_stamped.hpp"
#include "mesh_msgs/msg/mesh_vertex_costs_stamped.hpp"
#include "mesh_msgs/msg/mesh_vertex_costs_sparse_stamped.hpp"
#include "mesh_msgs/msg/mesh_vertex_tex_coords.hpp"
#include "mesh_msgs/msg/mesh_material.hpp"
#include "mesh_msgs/msg/mesh_texture.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"


namespace mesh_msgs_conversions
{

using Vec = lvr2::BaseVector<float>;
using PointBuffer = lvr2::PointBuffer;
using PointBufferPtr = lvr2::PointBufferPtr;

struct MaterialGroup
{
    int texture_index;
    unsigned char r;
    unsigned char g;
    unsigned char b;
    std::vector<unsigned int> faceBuffer;
};

typedef std::vector <boost::shared_ptr<MaterialGroup>> GroupVector;
typedef boost::shared_ptr <MaterialGroup> MaterialGroupPtr;

template<typename CoordType>
inline const mesh_msgs::msg::MeshGeometry toMeshGeometry(
    const std::shared_ptr<lvr2::BaseMesh<lvr2::BaseVector<CoordType>>> hem,
    const lvr2::VertexMap<lvr2::Normal<CoordType>>& normals = lvr2::DenseVertexMap<lvr2::Normal<CoordType>>())
{
  mesh_msgs::msg::MeshGeometry mesh_msg;
  mesh_msg.vertices.reserve(hem->numVertices());
  mesh_msg.faces.reserve(hem->numFaces());

  mesh_msg.vertex_normals.reserve(normals.numValues());

  lvr2::DenseVertexMap<size_t> new_indices;
  new_indices.reserve(hem->numVertices());

  size_t k = 0;
  for(auto vH : hem->vertices())
  {
    new_indices.insert(vH, k++);
    const auto& pi = hem->getVertexPosition(vH);
    geometry_msgs::msg::Point p;
    p.x = pi.x; p.y = pi.y; p.z = pi.z;
    mesh_msg.vertices.push_back(p);
  }

  for(auto fH : hem->faces())
  {
    mesh_msgs::msg::MeshTriangleIndices indices;
    auto vHs = hem->getVerticesOfFace(fH);
    indices.vertex_indices[0] = new_indices[vHs[0]];
    indices.vertex_indices[1] = new_indices[vHs[1]];
    indices.vertex_indices[2] = new_indices[vHs[2]];
    mesh_msg.faces.push_back(indices);
  }

  for(auto vH : hem->vertices())
  {
    const auto& n = normals[vH];
    geometry_msgs::msg::Point v;
    v.x = n.x; v.y = n.y; v.z = n.z;
    mesh_msg.vertex_normals.push_back(v);
  }

  return mesh_msg;
}

template<typename CoordType>
inline const mesh_msgs::msg::MeshGeometryStamped toMeshGeometryStamped(
    const std::shared_ptr<lvr2::BaseMesh<lvr2::BaseVector<CoordType>>> hem,
    const std::string& frame_id,
    const std::string& uuid,
    const lvr2::VertexMap<lvr2::Normal<CoordType>>& normals = lvr2::DenseVertexMap<lvr2::Normal<CoordType>>(),
    const rclcpp::Time& stamp = rclcpp::Time())
{
  mesh_msgs::msg::MeshGeometryStamped mesh_msg;
  mesh_msg.mesh_geometry = toMeshGeometry<CoordType>(hem, normals);
  mesh_msg.uuid = uuid;
  mesh_msg.header.frame_id = frame_id;
  mesh_msg.header.stamp = stamp;
  return mesh_msg;
}


template<typename CoordType>
inline const mesh_msgs::msg::MeshGeometry toMeshGeometry(
    const lvr2::HalfEdgeMesh<lvr2::BaseVector<CoordType>>& hem,
    const lvr2::VertexMap<lvr2::Normal<CoordType>>& normals = lvr2::DenseVertexMap<lvr2::Normal<CoordType>>())
{
  mesh_msgs::msg::MeshGeometry mesh_msg;
  mesh_msg.vertices.reserve(hem.numVertices());
  mesh_msg.faces.reserve(hem.numFaces());

  mesh_msg.vertex_normals.reserve(normals.numValues());

  lvr2::DenseVertexMap<size_t> new_indices;
  new_indices.reserve(hem.numVertices());

  size_t k = 0;
  for(auto vH : hem.vertices())
  {
    new_indices.insert(vH, k++);
    const auto& pi = hem.getVertexPosition(vH);
    geometry_msgs::msg::Point p;
    p.x = pi.x; p.y = pi.y; p.z = pi.z;
    mesh_msg.vertices.push_back(p);
  }

  for(auto fH : hem.faces())
  {
    mesh_msgs::msg::MeshTriangleIndices indices;
    auto vHs = hem.getVerticesOfFace(fH);
    indices.vertex_indices[0] = new_indices[vHs[0]];
    indices.vertex_indices[1] = new_indices[vHs[1]];
    indices.vertex_indices[2] = new_indices[vHs[2]];
    mesh_msg.faces.push_back(indices);
  }

  for(auto vH : hem.vertices())
  {
    const auto& n = normals[vH];
    geometry_msgs::msg::Point v;
    v.x = n.x; v.y = n.y; v.z = n.z;
    mesh_msg.vertex_normals.push_back(v);
  }

  return mesh_msg;
}

template<typename CoordType>
inline const mesh_msgs::msg::MeshGeometryStamped toMeshGeometryStamped(
    const lvr2::HalfEdgeMesh<lvr2::BaseVector<CoordType>>& hem,
    const std::string& frame_id,
    const std::string& uuid,
    const lvr2::VertexMap<lvr2::Normal<CoordType>>& normals = lvr2::DenseVertexMap<lvr2::Normal<CoordType>>(),
    const rclcpp::Time& stamp = rclcpp::Time())
{
    mesh_msgs::msg::MeshGeometryStamped mesh_msg;
    mesh_msg.mesh_geometry = toMeshGeometry<CoordType>(hem, normals);
    mesh_msg.uuid = uuid;
    mesh_msg.header.frame_id = frame_id;
    mesh_msg.header.stamp = stamp;
    return mesh_msg;
}

inline const mesh_msgs::msg::MeshVertexCosts toVertexCosts(
    const lvr2::VertexMap<float>& costs,
    const size_t num_values,
    const float default_value)
{
  mesh_msgs::msg::MeshVertexCosts costs_msg;
  costs_msg.costs.resize(num_values, default_value);
  for(auto vH : costs){
    costs_msg.costs[vH.idx()] = costs[vH];
  }
  return costs_msg;
}

inline const mesh_msgs::msg::MeshVertexCostsSparse toVertexCostsSparse(
    const lvr2::VertexMap<float>& costs,
    const float default_value
)
{
    mesh_msgs::msg::MeshVertexCostsSparse costs_msg;
    costs_msg.vertices.resize(costs.numValues());
    costs_msg.costs.resize(costs.numValues());
    costs_msg.default_value = default_value;

    std::transform(costs.begin(), costs.end(), costs_msg.vertices.begin(),
        [](const lvr2::VertexHandle& v)
        {
            return v.idx();
        }
    );
    std::transform(costs_msg.vertices.begin(), costs_msg.vertices.end(), costs_msg.costs.begin(),
        [&costs](const uint32_t idx)
        {
            return costs[lvr2::VertexHandle(idx)];
        }
    );
    return costs_msg;
}

inline const mesh_msgs::msg::MeshVertexCostsStamped toVertexCostsStamped(
    const lvr2::VertexMap<float>& costs,
    const size_t num_values,
    const float default_value,
    const std::string& name,
    const std::string& frame_id,
    const std::string& uuid,
    const rclcpp::Time& stamp = rclcpp::Time()
)
{
  mesh_msgs::msg::MeshVertexCostsStamped mesh_msg;
  mesh_msg.mesh_vertex_costs = toVertexCosts(costs, num_values, default_value);
  mesh_msg.uuid = uuid;
  mesh_msg.type = name;
  mesh_msg.header.frame_id = frame_id;
  mesh_msg.header.stamp = stamp;
  return mesh_msg;
}

inline const mesh_msgs::msg::MeshVertexCostsSparseStamped toVertexCostsSparseStamped(
    const lvr2::VertexMap<float>& costs,
    const float default_value,
    const std::string& name,
    const std::string& frame_id,
    const std::string& uuid,
    const rclcpp::Time& stamp = rclcpp::Time()
)
{
  mesh_msgs::msg::MeshVertexCostsSparseStamped msg;
  msg.mesh_vertex_costs = toVertexCostsSparse(costs, default_value);
  msg.uuid = uuid;
  msg.type = name;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return msg;
}

inline const mesh_msgs::msg::MeshVertexCosts toVertexCosts(
    const lvr2::DenseVertexMap<float>& costs)
{
    mesh_msgs::msg::MeshVertexCosts costs_msg;
    costs_msg.costs.reserve(costs.numValues());
    for(auto vH : costs){
        costs_msg.costs.push_back(costs[vH]);
    }
    return costs_msg;
}

inline const mesh_msgs::msg::MeshVertexCostsStamped toVertexCostsStamped(
    const lvr2::DenseVertexMap<float>& costs,
    const std::string& name,
    const std::string& frame_id,
    const std::string& uuid,
    const rclcpp::Time& stamp = rclcpp::Time()
    )
{
    mesh_msgs::msg::MeshVertexCostsStamped mesh_msg;
    mesh_msg.mesh_vertex_costs = toVertexCosts(costs);
    mesh_msg.uuid = uuid;
    mesh_msg.type = name;
    mesh_msg.header.frame_id = frame_id;
    mesh_msg.header.stamp = stamp;
    return mesh_msg;
}


bool fromMeshBufferToMeshGeometryMessage(
    const lvr2::MeshBufferPtr& buffer,
    mesh_msgs::msg::MeshGeometry& mesh_geometry
);

/// Convert lvr2::MeshBuffer to various messages for services
bool fromMeshBufferToMeshMessages(
    const lvr2::MeshBufferPtr& buffer,
    mesh_msgs::msg::MeshGeometry& mesh_geometry,
    mesh_msgs::msg::MeshMaterials& mesh_materials,
    mesh_msgs::msg::MeshVertexColors& mesh_vertex_colors,
    boost::optional<std::vector<mesh_msgs::msg::MeshTexture>&> texture_cache,
    std::string mesh_uuid
);

bool fromMeshGeometryToMeshBuffer(
    const std::shared_ptr<mesh_msgs::msg::MeshGeometry> mesh_geometry_ptr,
    lvr2::MeshBufferPtr& buffer_ptr
);

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::msg::MeshGeometry& mesh_geometry,
    lvr2::MeshBufferPtr& buffer_ptr
);

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::msg::MeshGeometry& mesh_geometry,
    lvr2::MeshBuffer& buffer);

/* TODO
void removeDuplicates(lvr2::MeshBuffer& buffer);
*/

/**
 * @brief Creates a LVR-MeshBufferPointer from a file
 *
 * @param path    Path to a MeshFile
 *
 * @return LVR-MeshBufferPointer
 */
bool readMeshBuffer(lvr2::MeshBufferPtr& buffer, string path);

/**
 * @brief Writes a LVR-MeshBufferPointer to a file
 *
 * @param mesh   LVR-MeshBufferPointer
 * @param path   Path to a MeshFile
 */
bool writeMeshBuffer(lvr2::MeshBufferPtr& mesh, string path);

bool fromPointCloud2ToPointBuffer(
    const sensor_msgs::msg::PointCloud2& cloud, PointBuffer& buffer);

/**
 * @brief converts lvr2::Pointbuffer to pointcloud2.
 *        Every channel is added as a pointfield.
 *
 * @param buffer the input lvr2::Pointbuffer
 * @param frame the frame of the converted pointcloud2
 * @param cloud the converted pointcloud2
 */
void PointBufferToPointCloud2(
    const lvr2::PointBufferPtr& buffer, 
    std::string frame, 
    std::shared_ptr<sensor_msgs::msg::PointCloud2>& cloud);

/**
 * @brief converts pointcloud2 to a newly created Pointerbuffer.
 *        Every pointfield is written into its own channel.
 *
 * @param cloud the input cloud
 * @param buffer the converted lvr2::Pointbuffer
 *
 * @return 
 */
void PointCloud2ToPointBuffer(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud,
    lvr2::PointBufferPtr& buffer);


/**
 * @brief Convert mesh_msgs::MeshGeometry to lvr2::MeshBuffer
 * @param message to be read
 * @param buffer to be returned
 * @return bool success status
 */
bool fromMeshGeometryMessageToMeshBuffer(
    const mesh_msgs::msg::MeshGeometry& mesh_geometry,
    const lvr2::MeshBufferPtr& buffer
);

} // end namespace

#endif /* MESH_MSGS_CONVERSIONS_H_ */
