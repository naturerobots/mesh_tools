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
 *  Types.hpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#pragma once

#include <vector>
#include <string>
#include <array>
#include <boost/optional.hpp>

namespace rviz_map_plugin
{
using boost::optional;
using std::array;
using std::string;
using std::vector;

/// Struct for normals
struct Normal
{
  float x;
  float y;
  float z;

  Normal(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
  {
  }
};

/// Struct for texture coordinates
struct TexCoords
{
  float u;
  float v;

  TexCoords(float _u, float _v) : u(_u), v(_v)
  {
  }
};

/// Struct for clusters
struct Cluster
{
  string name;
  vector<uint32_t> faces;

  Cluster(string n, vector<uint32_t> f) : name(n), faces(f)
  {
  }
};

/// Struct for vertices
struct Vertex
{
  float x;
  float y;
  float z;
};

/// Struct for faces
struct Face
{
  array<uint32_t, 3> vertexIndices;
};

/// Struct for geometry
struct Geometry
{
  vector<Vertex> vertices;
  vector<Face> faces;

  Geometry()
  {
  }

  Geometry(vector<float> v, vector<uint32_t> f)
  {
    for (uint32_t i = 0; i < v.size() / 3; i++)
    {
      Vertex vertex;
      vertex.x = v[i * 3 + 0];
      vertex.y = v[i * 3 + 1];
      vertex.z = v[i * 3 + 2];
      vertices.push_back(vertex);
    }

    for (uint32_t i = 0; i < f.size() / 3; i++)
    {
      Face face;
      face.vertexIndices[0] = f[i * 3 + 0];
      face.vertexIndices[1] = f[i * 3 + 1];
      face.vertexIndices[2] = f[i * 3 + 2];
      faces.push_back(face);
    }
  }
};

/// Struct for colors
struct Color
{
  float r;
  float g;
  float b;
  float a;

  Color()
  {
  }
  Color(float _r, float _g, float _b, float _a) : r(_r), g(_g), b(_b), a(_a)
  {
  }
};

/// Struct for textures
struct Texture
{
  uint32_t width;
  uint32_t height;
  uint8_t channels;
  vector<uint8_t> data;
  string pixelFormat;
};

/// Struct for materials
struct Material
{
  optional<uint32_t> textureIndex;
  Color color;
  vector<uint32_t> faceIndices;
};

}  // namespace rviz_map_plugin
