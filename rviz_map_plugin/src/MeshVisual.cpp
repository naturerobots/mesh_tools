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
 *  MeshVisual.cpp
 *
 *
 *  authors:
 *
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#include "MeshVisual.hpp"

#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreRenderOperation.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgrePixelFormat.h>

#include <limits>
#include <stdint.h>

namespace rviz_map_plugin
{
Ogre::ColourValue getRainbowColor1(float value)
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

MeshVisual::MeshVisual(rviz::DisplayContext* context, size_t displayID, size_t meshID, size_t randomID)
  : m_displayContext(context)
  , m_prefix(displayID)
  , m_postfix(meshID)
  , m_random(randomID)
  , m_vertex_normals_enabled(false)
  , m_vertex_colors_enabled(false)
  , m_materials_enabled(false)
  , m_texture_coords_enabled(false)
  , m_normalsScalingFactor(1)
{
  ROS_INFO("Creating MeshVisual %lu_TexturedMesh_%lu_%lu", m_prefix, m_postfix, m_random);

  // get or create the scene node
  Ogre::SceneManager* sceneManager = m_displayContext->getSceneManager();
  Ogre::SceneNode* rootNode = sceneManager->getRootSceneNode();

  std::stringstream strstream;
  strstream << "TexturedMeshScene" << m_random;
  std::string sceneId = strstream.str();
  if (sceneManager->hasSceneNode(sceneId))
  {
    // ROS_INFO("Attaching to scene: %s", sceneId);
    m_sceneNode = (Ogre::SceneNode*)(rootNode->getChild(sceneId));
  }
  else
  {
    // ROS_INFO("Creating new scene: %s", sceneId);
    m_sceneNode = rootNode->createChildSceneNode(sceneId);
  }

  // create manual objects and attach them to the scene node
  std::stringstream sstm;
  sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random;
  m_mesh = sceneManager->createManualObject(sstm.str());
  m_mesh->setDynamic(false);
  m_sceneNode->attachObject(m_mesh);

  std::stringstream sstmNormals;
  sstmNormals << m_prefix << "_Normals_" << m_postfix << "_" << m_random;
  m_normals = sceneManager->createManualObject(sstmNormals.str());
  m_normals->setDynamic(false);
  m_sceneNode->attachObject(m_normals);

  std::stringstream sstmTexturedMesh;
  sstmTexturedMesh << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random;
  m_texturedMesh = sceneManager->createManualObject(sstmTexturedMesh.str());
  m_texturedMesh->setDynamic(false);
  m_sceneNode->attachObject(m_texturedMesh);

  std::stringstream sstmNoTexCluMesh;
  sstmNoTexCluMesh << m_prefix << "_NoTexCluMesh_" << m_postfix << "_" << m_random;
  m_noTexCluMesh = sceneManager->createManualObject(sstmNoTexCluMesh.str());
  m_noTexCluMesh->setDynamic(false);
  m_sceneNode->attachObject(m_noTexCluMesh);

  std::stringstream sstmVertexCostsMesh;
  sstmVertexCostsMesh << m_prefix << "_VertexCostsMesh_" << m_postfix << "_" << m_random;
  m_vertexCostsMesh = sceneManager->createManualObject(sstmVertexCostsMesh.str());
  m_vertexCostsMesh->setDynamic(false);
  m_sceneNode->attachObject(m_vertexCostsMesh);
}

MeshVisual::~MeshVisual()
{
  ROS_INFO("Destroying MeshVisual %lu_TexturedMesh_%lu_%lu", m_prefix, m_postfix, m_random);

  reset();

  std::stringstream sstm;
  sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random;
  m_displayContext->getSceneManager()->destroyManualObject(sstm.str());

  std::stringstream sstmNormals;
  sstmNormals << m_prefix << "_Normals_" << m_postfix << "_" << m_random;
  m_displayContext->getSceneManager()->destroyManualObject(sstmNormals.str());

  std::stringstream sstmTexturedMesh;
  sstmTexturedMesh << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random;
  m_displayContext->getSceneManager()->destroyManualObject(sstmTexturedMesh.str());

  std::stringstream sstmNoTexCluMesh;
  sstmNoTexCluMesh << m_prefix << "_NoTexCluMesh_" << m_postfix << "_" << m_random;
  m_displayContext->getSceneManager()->destroyManualObject(sstmNoTexCluMesh.str());

  std::stringstream sstmVertexCostsMesh;
  sstmVertexCostsMesh << m_prefix << "_VertexCostsMesh_" << m_postfix << "_" << m_random;
  m_displayContext->getSceneManager()->destroyManualObject(sstmVertexCostsMesh.str());

  m_displayContext->getSceneManager()->destroySceneNode(m_sceneNode);
  sstm.str("");
  sstm.flush();
}

void MeshVisual::reset()
{
  ROS_INFO("Resetting MeshVisual %lu_TexturedMesh_%lu_%lu", m_prefix, m_postfix, m_random);

  std::stringstream sstm;

  sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "GeneralMaterial_";
  Ogre::MaterialManager::getSingleton().unload(sstm.str());
  Ogre::MaterialManager::getSingleton().remove(sstm.str());
  sstm.str("");
  sstm.clear();

  if (m_vertex_colors_enabled)
  {
    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "Material_" << 1;
    Ogre::MaterialManager::getSingleton().unload(sstm.str());
    Ogre::MaterialManager::getSingleton().remove(sstm.str());
    sstm.str("");
    sstm.clear();
  }

  sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "NormalMaterial";
  Ogre::MaterialManager::getSingleton().unload(sstm.str());
  Ogre::MaterialManager::getSingleton().remove(sstm.str());
  sstm.str("");
  sstm.clear();

  for (Ogre::MaterialPtr textureMaterial : m_textureMaterials)
  {
    Ogre::MaterialManager::getSingleton().unload(textureMaterial->getName());
    Ogre::MaterialManager::getSingleton().remove(textureMaterial->getName());
  }

  if (!m_noTexCluMaterial.isNull())
  {
    Ogre::MaterialManager::getSingleton().unload(m_noTexCluMaterial->getName());
    Ogre::MaterialManager::getSingleton().remove(m_noTexCluMaterial->getName());
  }

  if (!m_vertexCostMaterial.isNull())
  {
    Ogre::MaterialManager::getSingleton().unload(m_vertexCostMaterial->getName());
    Ogre::MaterialManager::getSingleton().remove(m_vertexCostMaterial->getName());
  }

  m_mesh->clear();
  m_normals->clear();
  m_texturedMesh->clear();
  m_noTexCluMesh->clear();
  m_vertexCostsMesh->clear();
  sstm.str("");
  sstm.flush();

  m_meshGeneralMaterial.setNull();
  m_normalMaterial.setNull();
  m_noTexCluMaterial.setNull();
  m_textureMaterials.clear();
  m_vertexCostMaterial.setNull();

  m_images.clear();

  m_vertex_colors_enabled = false;
  m_materials_enabled = false;
  m_texture_coords_enabled = false;
  m_textures_enabled = false;
  m_vertex_costs_enabled = false;
}

void MeshVisual::showWireframe(Ogre::Pass* pass, Ogre::ColourValue wireframeColor, float wireframeAlpha)
{
  pass->setAmbient(Ogre::ColourValue(wireframeColor.r, wireframeColor.g, wireframeColor.b, wireframeAlpha));
  pass->setDiffuse(Ogre::ColourValue(wireframeColor.r, wireframeColor.g, wireframeColor.b, wireframeAlpha));

  if (wireframeAlpha < 1.0)
  {
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(true);
  }
  pass->setPolygonMode(Ogre::PM_WIREFRAME);
  pass->setCullingMode(Ogre::CULL_NONE);
}

void MeshVisual::showFaces(Ogre::Pass* pass, Ogre::ColourValue facesColor, float facesAlpha,
                                   bool useVertexColors)
{
  pass->setDiffuse(Ogre::ColourValue(facesColor.r, facesColor.g, facesColor.b, facesAlpha));
  pass->setSelfIllumination(facesColor.r, facesColor.g, facesColor.b);

  if (useVertexColors)
  {
    pass->setLightingEnabled(false);
    pass->setDepthWriteEnabled(true);
  }
  else if (facesAlpha < 1.0)
  {
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
  }
  pass->setPolygonMode(Ogre::PM_SOLID);
  pass->setCullingMode(Ogre::CULL_NONE);
}

void MeshVisual::showNormals(Ogre::Pass* pass, Ogre::ColourValue normalsColor, float normalsAlpha)
{
  pass->setSelfIllumination(normalsColor.r, normalsColor.g, normalsColor.b);
  pass->setDiffuse(Ogre::ColourValue(normalsColor.r, normalsColor.g, normalsColor.b, normalsAlpha));
  if (normalsAlpha < 1.0)
  {
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(true);
  }
  pass->setPolygonMode(Ogre::PM_WIREFRAME);
  pass->setCullingMode(Ogre::CULL_NONE);
}

void MeshVisual::updateMaterial(bool showFaces, Ogre::ColourValue facesColor, float facesAlpha,
                                        bool useVertexColors, bool showVertexCosts, bool showTextures,
                                        bool showTexturedFacesOnly)
{
  // remove the faces pass
  if (!m_meshGeneralMaterial.isNull())
  {
    Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);
    if (tech->getPass("faces") != 0)
    {
      tech->removePass(tech->getPass("faces")->getIndex());
    }
  }

  m_texturedMesh->setVisible(false);
  m_noTexCluMesh->setVisible(false);
  m_vertexCostsMesh->setVisible(false);

  // if the material exists and the textures are not enabled
  // we can use the general mesh with the m_meshGeneralMaterial
  if (!m_meshGeneralMaterial.isNull() && !showTextures && !showVertexCosts)
  {
    Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);
    if (showFaces)
    {
      Ogre::Pass* pass = tech->createPass();
      pass->setName("faces");
      this->showFaces(pass, facesColor, facesAlpha, useVertexColors);
    }
  }

  // if there are vertex costs and the vertex cost are enabled
  // the mesh with the colors calculated from vertex costs is made visible
  if (m_vertex_costs_enabled && showVertexCosts)
  {
    m_vertexCostsMesh->setVisible(true);
  }

  // if there are materials or textures the mesh with texture coordinates that
  // uses the material and texture materials is made visible
  if ((m_materials_enabled || m_textures_enabled) && showTextures)
  {
    m_texturedMesh->setVisible(true);
    m_noTexCluMesh->setVisible(!showTexturedFacesOnly);
  }
}

void MeshVisual::updateMaterial(bool showWireframe, Ogre::ColourValue wireframeColor, float wireframeAlpha,
                                        bool showFaces, Ogre::ColourValue facesColor, float facesAlpha,
                                        bool useVertexColors, bool showVertexCosts, bool showTextures,
                                        bool showTexturedFacesOnly, bool showNormals, Ogre::ColourValue normalsColor,
                                        float normalsAlpha, float normalsScalingFactor)
{
  // remove all passes
  if (!m_meshGeneralMaterial.isNull())
  {
    m_meshGeneralMaterial->getTechnique(0)->removeAllPasses();
  }

  if (!m_normalMaterial.isNull())
  {
    m_normalMaterial->getTechnique(0)->removeAllPasses();
  }

  m_texturedMesh->setVisible(false);
  m_noTexCluMesh->setVisible(false);
  m_vertexCostsMesh->setVisible(false);

  // if the material exists and the textures are not enabled
  // we can use the general mesh with the m_meshGeneralMaterial
  if (!m_meshGeneralMaterial.isNull() && !showTextures && !showVertexCosts)
  {
    Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);

    if (showFaces)
    {
      this->showFaces(tech->createPass(), facesColor, facesAlpha, useVertexColors);
    }
  }

  // if there are vertex costs and the vertex cost are enabled
  // the mesh with the colors calculated from vertex costs is made visible
  if (m_vertex_costs_enabled && showVertexCosts)
  {
    m_vertexCostsMesh->setVisible(true);
  }

  // if there are materials or textures the mesh with texture coordinates that
  // uses the material and texture materials is made visible
  if ((m_materials_enabled || m_textures_enabled) && showTextures)
  {
    m_texturedMesh->setVisible(true);
    m_noTexCluMesh->setVisible(!showTexturedFacesOnly);  // TODO: dynamisch
  }

  if (showWireframe)
  {
    Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);
    this->showWireframe(tech->createPass(), wireframeColor, wireframeAlpha);
  }

  if (!m_normalMaterial.isNull())
  {
    if (showNormals)
    {
      Ogre::Technique* tech = m_normalMaterial->getTechnique(0);
      this->showNormals(tech->createPass(), normalsColor, normalsAlpha);
      updateNormals(normalsScalingFactor);
    }
  }
}

void MeshVisual::updateNormals(float scalingFactor)
{
  m_normalsScalingFactor = scalingFactor;
  enteringNormals(m_geometry, m_geometryNormals);
}

void MeshVisual::updateNormals(bool showNormals, Ogre::ColourValue normalsColor, float normalsAlpha)
{
  if (!m_normalMaterial.isNull())
  {
    m_normalMaterial->getTechnique(0)->removeAllPasses();

    if (showNormals)
    {
      Ogre::Technique* tech = m_normalMaterial->getTechnique(0);
      this->showNormals(tech->createPass(), normalsColor, normalsAlpha);
    }
  }
}

void MeshVisual::updateNormals(bool showNormals, Ogre::ColourValue normalsColor, float normalsAlpha,
                                       float scalingFactor)
{
  updateNormals(showNormals, normalsColor, normalsAlpha);
  updateNormals(scalingFactor);
}

void MeshVisual::updateWireframe(bool showWireframe, Ogre::ColourValue wireframeColor, float wireframeAlpha)
{
  if (!m_meshGeneralMaterial.isNull())
  {
    Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);

    if (tech->getPass("wireframe") != 0)
    {
      tech->removePass(tech->getPass("wireframe")->getIndex());
    }

    if (showWireframe)
    {
      Ogre::Pass* pass = tech->createPass();
      pass->setName("wireframe");
      this->showWireframe(pass, wireframeColor, wireframeAlpha);
    }
  }
}

void MeshVisual::enteringGeneralTriangleMesh(const Geometry& mesh)
{
  std::stringstream sstm;

  sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "GeneralMaterial_";

  m_meshGeneralMaterial = Ogre::MaterialManager::getSingleton().create(
      sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

  m_meshGeneralMaterial->getTechnique(0)->removeAllPasses();

  // start entering data
  m_mesh->clear();
  m_mesh->begin(sstm.str(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // write vertices
  for (size_t i = 0; i < mesh.vertices.size(); i++)
  {
    // write vertices
    m_mesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
  }

  // write triangles
  for (size_t i = 0; i < mesh.faces.size(); i++)
  {
    m_mesh->triangle(mesh.faces[i].vertexIndices[0], mesh.faces[i].vertexIndices[1], mesh.faces[i].vertexIndices[2]);
  }

  // finish entering data
  m_mesh->end();
}

void MeshVisual::enteringColoredTriangleMesh(const Geometry& mesh, const vector<Color>& vertexColors)
{
  if (m_meshGeneralMaterial.isNull())
  {
    std::stringstream sstm;
    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "GeneralMaterial_";

    m_meshGeneralMaterial = Ogre::MaterialManager::getSingleton().create(
        sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

    m_meshGeneralMaterial->getTechnique(0)->removeAllPasses();
  }

  // start entering data
  m_mesh->clear();
  m_mesh->begin(m_meshGeneralMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // write vertices
  // write vertex colors
  for (size_t i = 0; i < mesh.vertices.size(); i++)
  {
    // write vertices
    m_mesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);

    // write vertex colors
    m_mesh->colour(vertexColors[i].r, vertexColors[i].g, vertexColors[i].b, vertexColors[i].a);
  }

  // write triangles
  for (size_t i = 0; i < mesh.faces.size(); i++)
  {
    m_mesh->triangle(mesh.faces[i].vertexIndices[0], mesh.faces[i].vertexIndices[1], mesh.faces[i].vertexIndices[2]);
  }

  // finish entering data
  m_mesh->end();
}

void MeshVisual::enteringTriangleMeshWithVertexCosts(const Geometry& mesh, const vector<float>& vertexCosts,
                                                             int costColorType)
{
  // Calculate maximum value for vertex costs
  float maxCost = std::numeric_limits<float>::min();
  float minCost = std::numeric_limits<float>::max();
  for (float cost : vertexCosts)
  {
    if (std::isfinite(cost) && cost > maxCost)
      maxCost = cost;
    if (std::isfinite(cost) && cost < minCost)
      minCost = cost;
  }

  enteringTriangleMeshWithVertexCosts(mesh, vertexCosts, costColorType, minCost, maxCost);
}

void MeshVisual::enteringTriangleMeshWithVertexCosts(const Geometry& mesh, const vector<float>& vertexCosts,
                                                             int costColorType, float minCost, float maxCost)
{
  float range = maxCost - minCost;
  if (range <= 0)
  {
    ROS_ERROR("Illegal vertex cost limits!");
    return;
  }

  if (m_vertexCostMaterial.isNull())
  {
    std::stringstream sstm;
    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "VertexCostMaterial_";

    m_vertexCostMaterial = Ogre::MaterialManager::getSingleton().create(
        sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

    Ogre::Pass* pass = m_vertexCostMaterial->getTechnique(0)->getPass(0);
    pass->setCullingMode(Ogre::CULL_NONE);
    pass->setLightingEnabled(false);

    // start entering data
    m_vertexCostsMesh->clear();
    m_vertexCostsMesh->begin(m_vertexCostMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  }
  else
  {
    // start updating data
    m_vertexCostsMesh->beginUpdate(0);
  }

  // write vertices
  // write vertex colors
  // write vertex normals(if enabled)
  for (size_t i = 0; i < mesh.vertices.size(); i++)
  {
    // write vertices
    m_vertexCostsMesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);

    // write vertex colors that are calculated from the cost values
    float normalizedCost = (vertexCosts[i] - minCost) / range;
    normalizedCost = std::max(0.0f, normalizedCost);
    normalizedCost = std::min(1.0f, normalizedCost);
    m_vertexCostsMesh->colour(calculateColorFromCost(normalizedCost, costColorType));
  }

  // write triangles
  for (size_t i = 0; i < mesh.faces.size(); i++)
  {
    m_vertexCostsMesh->triangle(mesh.faces[i].vertexIndices[0], mesh.faces[i].vertexIndices[1],
                                mesh.faces[i].vertexIndices[2]);
  }

  // finish entering data
  m_vertexCostsMesh->end();
}

void MeshVisual::enteringTexturedTriangleMesh(const Geometry& mesh, const vector<Material>& materials,
                                                      const vector<TexCoords>& texCoords)
{
  std::stringstream sstm;
  sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "NoTexCluMaterial_";
  m_noTexCluMaterial = Ogre::MaterialManager::getSingleton().create(
      sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

  m_texturedMesh->setVisible(false);
  m_noTexCluMesh->setVisible(false);

  Ogre::Pass* pass = m_noTexCluMaterial->getTechnique(0)->getPass(0);
  pass->setCullingMode(Ogre::CULL_NONE);
  pass->setLightingEnabled(false);

  m_noTexCluMesh->begin(m_noTexCluMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  size_t noTexCluVertexCount = 0;

  size_t materialIndex = 0;

  for (auto& material : materials)
  {
    bool hasTexture = material.textureIndex ? true : false;

    // if the material has a texture, create an ogre texture and load the image
    if (hasTexture)
    {
      uint32_t textureIndex = *(material.textureIndex);
      std::stringstream sstm;
      sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "TextureMaterial_" << textureIndex;
      m_textureMaterials.push_back(Ogre::MaterialManager::getSingleton().create(
          sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true));

      // set some rendering options for textured clusters
      Ogre::Pass* pass = m_textureMaterials[textureIndex]->getTechnique(0)->getPass(0);
      // pass->setTextureFiltering(Ogre::TFO_NONE);
      pass->setCullingMode(Ogre::CULL_NONE);
      pass->setLightingEnabled(false);

      // check if image was already loaded
      // this is the case if the vector of images doesn't contain this element yet or
      // if the image was only default constructed, in which case its width will be 0
      if (m_images.size() < textureIndex + 1 || m_images[textureIndex].getWidth() == 0)
      {
        ROS_DEBUG("Texture with index %u not loaded yet", textureIndex);
      }
      else
      {
        loadImageIntoTextureMaterial(textureIndex);
      }
    }

    if (hasTexture)
    {
      uint32_t textureIndex = *(material.textureIndex);
      // start entering data
      m_texturedMesh->begin(m_textureMaterials[textureIndex]->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

      // write vertices for each triangle
      // write texture coordinates

      size_t triangleVertexCount = 0;
      for (size_t i = 0; i < material.faceIndices.size(); i++)
      {
        uint32_t faceIndex = material.faceIndices[i];
        // write three triangle vertices
        for (size_t j = 0; j < 3; j++)
        {
          uint32_t vertexIndex = mesh.faces[faceIndex].vertexIndices[j];
          // write vertex positions
          m_texturedMesh->position(mesh.vertices[vertexIndex].x, mesh.vertices[vertexIndex].y,
                                   mesh.vertices[vertexIndex].z);
          // write texture coordinates
          m_texturedMesh->textureCoord(texCoords[vertexIndex].u, 1 - texCoords[vertexIndex].v);
        }
        // write the three triangle vertex indices
        m_texturedMesh->triangle(triangleVertexCount, triangleVertexCount + 1, triangleVertexCount + 2);
        triangleVertexCount += 3;
      }

      // finish entering data
      m_texturedMesh->end();
    }
    else
    {
      // write vertices for each triangle to enable a coloring for each triangle
      // write triangle colors as vertex colours

      size_t triangleVertexCount = 0;
      for (size_t i = 0; i < material.faceIndices.size(); i++)
      {
        uint32_t faceIndex = material.faceIndices[i];
        // write three triangle vertices
        for (size_t j = 0; j < 3; j++)
        {
          int vertexIndex = mesh.faces[faceIndex].vertexIndices[j];
          // write vertex positions
          m_noTexCluMesh->position(mesh.vertices[vertexIndex].x, mesh.vertices[vertexIndex].y,
                                   mesh.vertices[vertexIndex].z);

          // write triangle colors
          m_noTexCluMesh->colour(material.color.r, material.color.g, material.color.b, material.color.a);
        }
        // write the three triangle vertex indices
        m_noTexCluMesh->triangle(noTexCluVertexCount, noTexCluVertexCount + 1, noTexCluVertexCount + 2);
        noTexCluVertexCount += 3;
      }
    }
  }

  m_noTexCluMesh->end();
}

void MeshVisual::enteringNormals(const Geometry& mesh, const vector<Normal>& normals)
{
  if (!m_vertex_normals_enabled)
  {
    return;
  }

  std::stringstream sstm;
  if (m_normalMaterial.isNull())
  {
    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "NormalMaterial";
    m_normalMaterial = Ogre::MaterialManager::getSingleton().create(
        sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
    m_normalMaterial->getTechnique(0)->removeAllPasses();

    // Create pointNormals
    m_normals->clear();
    m_normals->begin(sstm.str(), Ogre::RenderOperation::OT_LINE_LIST);
  }
  else
  {
    m_normals->beginUpdate(0);
  }

  // Vertices
  for (size_t i = 0; i < mesh.vertices.size(); i++)
  {
    m_normals->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
    m_normals->position(mesh.vertices[i].x + m_normalsScalingFactor * normals[i].x,
                        mesh.vertices[i].y + m_normalsScalingFactor * normals[i].y,
                        mesh.vertices[i].z + m_normalsScalingFactor * normals[i].z);
    // add line to index buffer
    m_normals->index(2 * i);
    m_normals->index(2 * i + 1);
  }
  m_normals->end();
}

bool MeshVisual::setGeometry(const Geometry& mesh)
{
  reset();

  m_geometry = mesh;

  // default: vertex colors are optional and therefore disabled
  m_vertex_colors_enabled = false;

  // default: textures and texture_coords are optional and therefore disabled
  m_textures_enabled = false;
  m_texture_coords_enabled = false;

  // default: vertex normals are optional and therefore disabled
  m_vertex_normals_enabled = false;

  // default: vertex costs are optional and therefore disabled
  m_vertex_costs_enabled = false;

  // check if there are enough vertices given
  if (mesh.vertices.size() < 3)
  {
    ROS_WARN("Received not enough vertices, can't create mesh!");
    return false;
  }

  // defines the buffer sizes
  int vertex_count = mesh.vertices.size();
  int index_count = mesh.faces.size() * 3;

  // avoid memory reallocation
  m_mesh->estimateVertexCount(vertex_count);
  m_mesh->estimateIndexCount(index_count);

  // entering a general triangle mesh into the internal buffer
  enteringGeneralTriangleMesh(mesh);

  return true;
}

bool MeshVisual::setNormals(const vector<Normal>& normals)
{
  // vertex normals
  // check if there are vertex normals for each vertex
  if (normals.size() == m_geometry.vertices.size())
  {
    ROS_INFO("Received %lu vertex normals.", normals.size());
    m_vertex_normals_enabled = true;
  }
  else if (normals.size() > 0)
  {
    ROS_WARN("Received not as much vertex normals as vertices, ignoring vertex normals!");
    return false;
  }

  m_geometryNormals = normals;

  // avoid memory reallocation
  m_normals->estimateVertexCount(m_geometry.vertices.size() * 2);
  m_normals->estimateIndexCount(m_geometry.vertices.size() * 2);

  // entering the normals into the internal buffer
  if (m_vertex_normals_enabled)
  {
    enteringNormals(m_geometry, normals);
  }

  return true;
}

bool MeshVisual::setVertexColors(const vector<Color>& vertexColors)
{
  // check if there are vertex colors for each vertex
  if (vertexColors.size() == m_geometry.vertices.size())
  {
    ROS_INFO("Received %lu vertex colors.", vertexColors.size());
    m_vertex_colors_enabled = true;
  }
  else
  {
    ROS_WARN("Received not as much vertex colors as vertices, ignoring the vertex colors!");
    return false;
  }

  enteringColoredTriangleMesh(m_geometry, vertexColors);

  return true;
}

bool MeshVisual::setVertexCosts(const vector<float>& vertexCosts)
{
  return setVertexCosts(vertexCosts, 0);
}

bool MeshVisual::setVertexCosts(const std::vector<float>& vertexCosts, int costColorType)
{
  //   // check if these MeshVertexCosts belong to the current mesh and were not already loaded
  //   if (m_meshUuid != vertexCostsMsg->uuid)
  //   {
  //     ROS_WARN("Can't add vertex costs, uuids do not match.");
  //     return false;
  //   }

  // check if there are vertex costs for each vertex
  if (vertexCosts.size() == m_geometry.vertices.size())
  {
    ROS_DEBUG("Received %lu vertex costs.", vertexCosts.size());
    m_vertex_costs_enabled = true;
  }
  else
  {
    ROS_WARN("Received not as much vertex costs as vertices, ignoring the vertex costs!");
    return false;
  }

  enteringTriangleMeshWithVertexCosts(m_geometry, vertexCosts, costColorType);

  //   m_vertexCostsUuid = vertexCostsMsg->uuid;

  return true;
}

bool MeshVisual::setVertexCosts(const std::vector<float>& vertexCosts, int costColorType, float minCost,
                                        float maxCost)
{
  //   // check if these MeshVertexCosts belong to the current mesh and were not already loaded
  //   if (m_meshUuid != vertexCostsMsg->uuid)
  //   {
  //     ROS_WARN("Can't add vertex costs, uuids do not match.");
  //     return false;
  //   }

  // check if there are vertex costs for each vertex
  if (vertexCosts.size() == m_geometry.vertices.size())
  {
    ROS_DEBUG("Received %lu vertex costs.", vertexCosts.size());
    m_vertex_costs_enabled = true;
  }
  else
  {
    ROS_WARN("Received not as much vertex costs as vertices, ignoring the vertex costs!");
    return false;
  }

  enteringTriangleMeshWithVertexCosts(m_geometry, vertexCosts, costColorType, minCost, maxCost);

  //   m_vertexCostsUuid = vertexCostsMsg->uuid;

  return true;
}

bool MeshVisual::setMaterials(const vector<Material>& materials, const vector<TexCoords>& texCoords)
{
  // check if there is a material index for each cluster
  if (materials.size() >= 0)
  {
    ROS_INFO("Received %lu materials.", materials.size());
    m_materials_enabled = true;  // enable materials
  }
  else
  {
    ROS_WARN("Received zero materials, ignoring materials!");
    return false;
  }

  // texture coords
  // check if there are texture coords for each vertex
  if (texCoords.size() == m_geometry.vertices.size())
  {
    ROS_INFO("Received %lu texture coords.", texCoords.size());
    m_texture_coords_enabled = true;  // enable texture coords
    m_textures_enabled = true;        // enable textures
  }
  else if (texCoords.size() > 0)
  {
    ROS_WARN("Received not as much texture coords as vertices, ignoring texture coords!");
  }

  enteringTexturedTriangleMesh(m_geometry, materials, texCoords);

  return true;
}

bool MeshVisual::addTexture(Texture& texture, uint32_t textureIndex)
{
  uint32_t width = texture.width;
  uint32_t height = texture.height;
  uint32_t step = texture.channels;

  uint32_t dataSize = width * height * step;

  Ogre::PixelFormat pixelFormat = getOgrePixelFormatFromRosString(texture.pixelFormat);

  Ogre::Image image = Ogre::Image();
  image.loadDynamicImage(texture.data.data(), width, height, 1, pixelFormat, false);
  m_images.insert(m_images.begin() + textureIndex, image);

  if (m_textureMaterials.size() >= textureIndex + 1)
  {
    loadImageIntoTextureMaterial(textureIndex);
    return true;
  }
  else
  {
    ROS_WARN("Can't load image into texture material, material does not exist!");
    return false;
  }
}

Ogre::PixelFormat MeshVisual::getOgrePixelFormatFromRosString(std::string encoding)
{
  if (encoding == "rgba8")
  {
    return Ogre::PF_BYTE_RGBA;
  }
  else if (encoding == "rgb8")
  {
    return Ogre::PF_BYTE_RGB;
  }

  ROS_WARN("Unknown texture encoding! Using Ogre::PF_UNKNOWN");
  return Ogre::PF_UNKNOWN;
}

void MeshVisual::loadImageIntoTextureMaterial(size_t textureIndex)
{
  std::stringstream textureNameStream;
  textureNameStream << m_prefix << "_Texture" << textureIndex << "_" << m_postfix << "_" << m_random;

  Ogre::TexturePtr texturePtr = Ogre::TextureManager::getSingleton().createManual(
      textureNameStream.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
      m_images[textureIndex].getWidth(), m_images[textureIndex].getHeight(), 0, m_images[textureIndex].getFormat());

  texturePtr->loadImage(m_images[textureIndex]);

  Ogre::Pass* pass = m_textureMaterials[textureIndex]->getTechnique(0)->getPass(0);
  pass->removeAllTextureUnitStates();
  pass->createTextureUnitState()->addFrameTextureName(textureNameStream.str());
}

Ogre::ColourValue MeshVisual::calculateColorFromCost(float cost, int costColorType)
{
  Ogre::ColourValue color;

  switch (costColorType)
  {
    case 0:  // rainbow
      return getRainbowColor1(cost);
    case 1:  // red green
      // calculate a color that is green for 0, yellow for 0.5 and red for 1
      color.r = cost * 2;
      color.r = color.r > 1.0f ? 1.0f : color.r;
      color.g = (1.0f - cost) * 2;
      color.g = color.g > 1.0f ? 1.0f : color.g;
      color.b = 0.0f;
      color.a = 1.0;
      return color;
    default:
      break;
  }
  // default
  return getRainbowColor1(cost);
}

void MeshVisual::setFramePosition(const Ogre::Vector3& position)
{
  m_sceneNode->setPosition(position);
}

void MeshVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  m_sceneNode->setOrientation(orientation);
}

}  // end namespace rviz_map_plugin
