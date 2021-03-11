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
 *  ClusterLabelVisual.cpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 */

#include <ClusterLabelVisual.hpp>

#include <rviz/display_context.h>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMesh.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSubMesh.h>

namespace rviz_map_plugin
{
ClusterLabelVisual::ClusterLabelVisual(rviz::DisplayContext* context, std::string labelId)
  : m_displayContext(context), m_labelId(labelId)
{
}

ClusterLabelVisual::ClusterLabelVisual(rviz::DisplayContext* context, std::string labelId,
                                       std::shared_ptr<Geometry> geometry)
  : m_displayContext(context), m_labelId(labelId), m_geometry(geometry)
{
  // Get or create scene node
  Ogre::SceneManager* sceneManager = m_displayContext->getSceneManager();
  Ogre::SceneNode* rootNode = sceneManager->getRootSceneNode();

  std::stringstream strstream;
  strstream << "ClusterLabelScene";
  std::string sceneId = strstream.str();

  if (sceneManager->hasSceneNode(sceneId))
  {
    m_sceneNode = (Ogre::SceneNode*)(rootNode->getChild(sceneId));
  }
  else
  {
    m_sceneNode = rootNode->createChildSceneNode(sceneId);
  }

  // Retrieve or create the mesh and attach it to the scene node
  m_mesh = Ogre::MeshManager::getSingleton().getByName("ClusterLabelMesh", "General");
  if (m_mesh.isNull() && geometry)
  {
    m_mesh = Ogre::MeshManager::getSingleton().createManual("ClusterLabelMesh", "General");

    // Create the vertex data structure
    m_mesh->sharedVertexData = new Ogre::VertexData;
    m_mesh->sharedVertexData->vertexCount = geometry->vertices.size();

    // Declare how the vertices will be represented
    Ogre::VertexDeclaration* decl = m_mesh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;

    // The first three floats of each vertex represent the position
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // Create the vertex buffer
    Ogre::HardwareVertexBufferSharedPtr vertexBuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        offset, m_mesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC);

    // Lock the buffer so we can get exclusive access to its data
    float* vertices = static_cast<float*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

    // Write the mesh data into the buffer
    for (int i = 0; i < m_mesh->sharedVertexData->vertexCount; i++)
    {
      vertices[(i * 3) + 0] = geometry->vertices[i].x;
      vertices[(i * 3) + 1] = geometry->vertices[i].y;
      vertices[(i * 3) + 2] = geometry->vertices[i].z;
    }

    // Unlock the buffer
    vertexBuffer->unlock();

    // Attach the vertex buffer to the mesh
    m_mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vertexBuffer);

    // Set the bounds of the mesh
    // TODO: Calculate the correct bounding box
    m_mesh->_setBounds(Ogre::AxisAlignedBox::EXTENT_INFINITE);

    // Notify the mesh that we're all ready
    m_mesh->load();

    // Create an entity of the mesh and add it to the scene
    Ogre::Entity* entity = sceneManager->createEntity("ClusterLabelEntity", "ClusterLabelMesh", "General");
    entity->setMaterialName("CustomMaterial", "General");
    m_sceneNode->attachObject(entity);
  }

  // Create a submesh and a custom material for it
  if (!m_mesh.isNull())
  {
    m_subMesh = m_mesh->createSubMesh(m_labelId);
    m_subMesh->useSharedVertices = true;
    std::stringstream sstm;
    sstm << "ClusterLabel_Material_" << m_labelId;
    m_material = Ogre::MaterialManager::getSingleton().create(
        sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

    m_subMesh->setMaterialName(m_material->getName(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    m_material->getTechnique(0)->removeAllPasses();
    m_material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    m_material->setSceneBlending(Ogre::SBT_ADD);
    m_material->setDepthWriteEnabled(false);

    initMaterial();
  }
}

ClusterLabelVisual::~ClusterLabelVisual()
{
  reset();

  if (!m_mesh.isNull())
  {
    ROS_DEBUG("ClusterLabelVisual::~ClusterLabelVisual: Destroying SubMesh: %s", m_labelId.c_str());

    try
    {
      m_mesh->destroySubMesh(m_labelId);
    }
    catch (...)
    {
      ROS_ERROR("Exception in Visual destructor");
    }
  }

  if (m_sceneNode->numAttachedObjects() == 0)
  {
    ROS_INFO("ClusterLabelVisual::~ClusterLabelVisual: Delete scene node");
    m_displayContext->getSceneManager()->destroySceneNode(m_sceneNode);
  }
}

void ClusterLabelVisual::reset()
{
  if (!m_material.isNull())
  {
    Ogre::MaterialManager::getSingleton().unload(m_material->getName());
    Ogre::MaterialManager::getSingleton().remove(m_material->getName());
  }
}

void ClusterLabelVisual::setGeometry(std::shared_ptr<Geometry> geometry)
{
}

void ClusterLabelVisual::setFacesInCluster(const std::vector<uint32_t>& faces)
{
  m_faces = faces;

  if (!m_geometry)
  {
    ROS_WARN("ClusterLabelVisual::setFacesInCluster: MeshGeometry not set!");
    return;
  }

  // don't draw the cluster if there are no faces in it
  if (faces.empty())
  {
    m_subMesh->indexData->indexBuffer.setNull();
    m_subMesh->indexData->indexCount = 0;
    m_subMesh->indexData->indexStart = 0;
    m_material->getTechnique(0)->removeAllPasses();
    ROS_DEBUG("ClusterLabelVisual::setFacesInCluster: faces empty!");
    return;
  }

  // if there are faces and there are no passes, create a pass to draw the cluster
  if (m_material->getTechnique(0)->getNumPasses() == 0)
  {
    m_material->getTechnique(0)->createPass();
    m_material->setDiffuse(m_color);
    m_material->setSelfIllumination(m_color);

    initMaterial();
  }

  size_t indexCount = faces.size() * 3;

  // Create the index buffer
  Ogre::HardwareIndexBufferSharedPtr indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_32BIT, indexCount, Ogre::HardwareBuffer::HBU_WRITE_ONLY);

  // Lock the buffer so we can get exclusive access to its data
  uint32_t* indices = static_cast<uint32_t*>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_WRITE_ONLY));

  // Define the triangles
  for (int i = 0; i < faces.size(); i++)
  {
    uint32_t faceId = faces[i];
    indices[i * 3 + 0] = m_geometry->faces[faceId].vertexIndices[0];
    indices[i * 3 + 1] = m_geometry->faces[faceId].vertexIndices[1];
    indices[i * 3 + 2] = m_geometry->faces[faceId].vertexIndices[2];
  }

  // Unlock the buffer
  indexBuffer->unlock();

  // Attach the index buffer to the submesh
  m_subMesh->indexData->indexBuffer = indexBuffer;
  m_subMesh->indexData->indexCount = indexCount;
  m_subMesh->indexData->indexStart = 0;
}

void ClusterLabelVisual::setColor(Ogre::ColourValue facesColor, float alpha)
{
  if (!m_material.isNull())
  {
    facesColor.a = alpha;
    m_material->setDiffuse(facesColor);
    m_material->setSelfIllumination(facesColor);
    m_color = facesColor;
  }
}

void ClusterLabelVisual::initMaterial()
{
  m_material->setCullingMode(Ogre::CULL_NONE);
  m_material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  m_material->setDepthWriteEnabled(false);
}

}  // End namespace rviz_map_plugin
