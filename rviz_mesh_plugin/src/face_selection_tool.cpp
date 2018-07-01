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
 *  face_selection_tool.cpp
 *
 *	authors:  Henning Deeken <hdeeken@uos.de>
 *            Sebastian Pütz <spuetz@uos.de>
 *            Tristan Igelbrink <tigelbri@uos.de>
 *            Johannes Heitmann <joheitma@uos.de>
 *            Marcel Mrozinski  <mmronzs@uos.de
 *
 */


#include "face_selection_tool.h"
#include <rviz/properties/ros_topic_property.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_mesh_plugin::FaceSelectionTool, rviz::Tool )

namespace rviz_mesh_plugin
{
const float FaceSelectionTool::BOX_SIZE_TOLERANCE = 0.0001;
const size_t FaceSelectionTool::MAXIMUM_PICKED_FACES = 10000;

FaceSelectionTool::FaceSelectionTool()
{
  shortcut_key_ = 'l';
  num_results = 10;
  has_mesh = false;
  m_singleSelect = false;
  m_singleDeselect = false;

  mesh_topic = new rviz::RosTopicProperty(
    "Mesh Topic",
    "segment_mesh",
    QString::fromStdString(ros::message_traits::datatype<mesh_msgs::TriangleMeshStamped>()),
    "Mesh topic to subscribe to.",
    getPropertyContainer(),
    SLOT( updateTopic() ),
    this
  );
}

FaceSelectionTool::~FaceSelectionTool()
{
  for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin();
    it != m_goalFaces.end(); it++)
  {
      it->second.clear();
  }
  m_goalFaces.clear();
  scene_manager->destroyManualObject("ReferenceMesh2");
  scene_manager->destroyManualObject("SegmentedMesh2");
  scene_manager->destroySceneNode(scene_node);
}

void FaceSelectionTool::updateTopic(){
  //if(mesh_topic->getTopic() != QString::fromStdString(mesh_sub.getTopic())){
    ROS_INFO("updated topic");
    mesh_sub.shutdown();
    mesh_sub = n.subscribe(mesh_topic->getTopic().toStdString(), 1, &FaceSelectionTool::meshCb, this);
    reference_mesh->clear();
    has_mesh = false;
    context_->queueRender();
  //}
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
void FaceSelectionTool::onInitialize()
{
  ROS_INFO("Call Init");

  initNode();
  initOgre();
  updateTopic();
}

void FaceSelectionTool::initOgre()
{
  scene_manager = context_->getSceneManager();
  Ogre::SceneNode* rootNode = scene_manager->getRootSceneNode();
  scene_node = rootNode->createChildSceneNode();

  // create reference mesh object and pass
  reference_mesh = scene_manager->createManualObject("ReferenceMesh2");
  reference_mesh->setDynamic(false);
  reference_mesh->setVisible(true);
  scene_node->attachObject(reference_mesh);

  reference_mesh_material = Ogre::MaterialManager::getSingleton()
    .create("ReferenceMeshMaterial2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
  Ogre::Technique* ref_tech = reference_mesh_material->getTechnique(0);
  ref_tech->removeAllPasses();

  Ogre::Pass* ref_pass = ref_tech->createPass();
  ref_pass->setAmbient( Ogre::ColourValue(reference_color_r, reference_color_g, reference_color_b, reference_color_a) );
  ref_pass->setDiffuse(0,0,0,reference_color_a);
  ref_pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  ref_pass->setDepthWriteEnabled(false);
  ref_pass->setPolygonMode(Ogre::PM_SOLID);
  ref_pass->setCullingMode(Ogre::CULL_NONE);

  // create segmented mesh object and pass
  segment_mesh = scene_manager->createManualObject("SegmentedMesh2");
  segment_mesh->setDynamic(false);
  scene_node->attachObject(segment_mesh);

  segment_mesh_material = Ogre::MaterialManager::getSingleton()
    .create("SegmentMatrial2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
  Ogre::Technique* seg_tech = segment_mesh_material->getTechnique(0);
  seg_tech->removeAllPasses();
  Ogre::Pass* seg_pass = seg_tech->createPass();

  seg_pass->setAmbient( Ogre::ColourValue(segment_color_r, segment_color_g, segment_color_b, segment_color_a) );
  seg_pass->setDiffuse(0,0,0, segment_color_a);
  seg_pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  seg_pass->setDepthWriteEnabled(false);
  seg_pass->setPolygonMode(Ogre::PM_SOLID);
  seg_pass->setCullingMode(Ogre::CULL_NONE);
}

void FaceSelectionTool::initNode()
{
  mesh_sub = n.subscribe( "segment_mesh", 1, &FaceSelectionTool::meshCb, this);
  mesh_pub = n.advertise<mesh_msgs::TriangleMeshStamped>( "segmented_mesh", 1, true);

  id_pub = n.advertise<std_msgs::Int32>( "selected_face_id", 1, true);
  goal_pub = n.advertise<geometry_msgs::PoseStamped>( "goal", 1, true );
}


void FaceSelectionTool::meshCb(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh)
{
  if(!has_mesh)
  {
    setReferenceMesh(mesh->mesh);
    setTransform(*mesh);
    has_mesh = true;
  }
}

void FaceSelectionTool::activate()
{
}

void FaceSelectionTool::deactivate()
{
}

void FaceSelectionTool::setTransform(const mesh_msgs::TriangleMeshStamped &mesh){
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if(!context_->getFrameManager()->getTransform(mesh.header.frame_id,
                                                  mesh.header.stamp,
                                                  position, orientation))
    {
        ROS_ERROR("Error transforming from frame '%s' to the fixed_frame",
        mesh.header.frame_id.c_str());
        return;
    }

    scene_node->setPosition(position);
    scene_node->setOrientation(orientation);
}

void FaceSelectionTool::setReferenceMesh( mesh_msgs::TriangleMesh mesh )
{
  clearSelection();
  reference_mesh->begin("ReferenceMeshMaterial2", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for ( size_t i = 0; i < mesh.vertices.size(); i++ )
  {
    reference_mesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
  }

  for ( size_t i = 0; i < mesh.triangles.size(); i++ )
  {
    reference_mesh->triangle( mesh.triangles[i].vertex_indices[0], mesh.triangles[i].vertex_indices[1],  mesh.triangles[i].vertex_indices[2] );
  }

  reference_mesh->end();
}

void FaceSelectionTool::getSegmentMesh(mesh_msgs::TriangleMesh &mesh) //size_t goalSection, std::string regionLabel
{
  size_t numSections = segment_mesh->getNumSections();
  //ROS_INFO("Mesh has %d sections, we only take the first", numSections);

  size_t vertexCount;
  Ogre::Vector3* vertices;
  size_t indexCount;
  unsigned long* indices;

  if(numSections > 0)
  {
    getRawManualObjectData(segment_mesh, 0, vertexCount, vertices, indexCount, indices);
    //ROS_INFO("#vert %d #ind %d", vertexCount, indexCount );

    geometry_msgs::Point vertex;
    mesh_msgs::TriangleIndices index;
    for ( size_t i = 0; i < vertexCount; i++ )
    {
      vertex.x = vertices[i].x;
      vertex.y = vertices[i].y;
      vertex.z = vertices[i].z;
      mesh.vertices.push_back(vertex);
    }

    for ( size_t i = 0; i < indexCount; i+=3 )
    {
      index.vertex_indices[0] = indices[i];
      index.vertex_indices[1] = indices[i+1];
      index.vertex_indices[2] = indices[i+2];
      mesh.triangles.push_back(index);
    }
  }
}

void FaceSelectionTool::clearSelection()
{
  segment_mesh->clear();
  for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
  {
    it->second.clear();
  }
  m_goalFaces.clear();
}

bool FaceSelectionTool::areFacesSelected()
{
  return (m_goalFaces.size() > 0);
}

void FaceSelectionTool::updateSelectionMesh()
{
  size_t facesSize = 0;
  size_t vertexCount = 0;
  Ogre::Vector3* vertices;
  size_t indexCount = 0;
  unsigned long* indices;
  segment_mesh->clear();
  Ogre::ManualObject* mesh = context_->getSceneManager()->getManualObject("ReferenceMesh2");
  segment_mesh->begin("SegmentMatrial2", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
  {
      getRawManualObjectData(mesh, it->first, vertexCount, vertices, indexCount, indices);
      facesSize += it->second.size();

      for (size_t j = 0; j < it->second.size(); j++)
      {
        segment_mesh->position(vertices[indices[it->second[j]]].x,
                                  vertices[indices[it->second[j]]].y,
                                  vertices[indices[it->second[j]]].z);
        segment_mesh->position(vertices[indices[it->second[j] + 1]].x,
                                  vertices[indices[it->second[j] + 1]].y,
                                  vertices[indices[it->second[j] + 1]].z);
        segment_mesh->position(vertices[indices[it->second[j] + 2]].x,
                                  vertices[indices[it->second[j] + 2]].y,
                                  vertices[indices[it->second[j] + 2]].z);
      }
      delete[] vertices;
      delete[] indices;
  }

  for (size_t j = 0; j < facesSize; j++)
  {
      segment_mesh->triangle(3 * j, 3 * j + 2, 3 * j + 1);
  }
  segment_mesh->end();
}

// Handling key events to label marked faces or to get db structure
int FaceSelectionTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel* panel)
{
  if (event->key() == Qt::Key_K)
  {
     ROS_INFO("IDS..");
     for (std::map<size_t, std::vector<size_t> >::iterator it = m_goalFaces.begin(); it != m_goalFaces.end(); it++)
     {
      for (size_t j = 0; j < it->second.size(); j++)
      {
        ROS_INFO("ID: %lu", it->second[j]);
      }
    }
  }
    /*
    mesh_msgs::TriangleMeshStamped mesh;
    getSegmentMesh(mesh.mesh);
    mesh.header.frame_id = "world";
    mesh.header.stamp = ros::Time::now();
    mesh_pub.publish( mesh );
    */

  // if 'r' is pressed clear the current selection of faces
  if (event->key() == Qt::Key_R)
  {
    clearSelection();
  }

  if (event->key() == Qt::Key_T)
  {
    reference_mesh->setVisible( !reference_mesh->isVisible() );
    segment_mesh->setVisible( !segment_mesh->isVisible() );
  }

  return Render;
}

// Handling mouse event and mark the clicked faces
int FaceSelectionTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{

  if (event.leftDown())
  {
    m_singleSelect = true;
    selectSingleFace(event);
  }

  else if (event.leftUp())
  {
    m_singleSelect = false;
    selectSingleFace(event);
  }

  else if (m_singleSelect)
  {
    selectSingleFace(event);
  }

  else if (event.rightDown())
  {
    m_singleDeselect = true;
    deselectSingleFace(event);
  }

  else if (event.rightUp())
  {
      m_singleDeselect = false;
      deselectSingleFace(event);
  }

  else if (m_singleDeselect)
  {
      deselectSingleFace(event);
  }

  return Render;
}

// test whether a single ray intersects with the reference mesh
// by checking if the reference mesh is within the num_results closest targets
// num_results must be >= 2 to allow labeling direct sight, higher allows to label trough objects... unsure if that is desirable
bool FaceSelectionTool::singleRayQuery(rviz::ViewportMouseEvent& event, int num_results, Ogre::Ray& ray)
{
  ray = event.viewport->getCamera()->getCameraToViewportRay((float) event.x / event.viewport->getActualWidth(),
                                                            (float) event.y / event.viewport->getActualHeight());
  Ogre::RaySceneQuery* query = context_->getSceneManager()->createRayQuery(ray, Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
  query-> setSortByDistance(true, num_results);
  query->execute();
  Ogre::RaySceneQueryResult &results = query->getLastResults();

  for (size_t i = 0; i < results.size(); i++)
  {

    Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(results[i].movable);
    //ROS_INFO("%s", mesh->getName().c_str() );
    if (mesh->getName().find("ReferenceMesh") != std::string::npos)
    {
      return true;
    }
  }
  return false;
}

void FaceSelectionTool::selectSingleFace(rviz::ViewportMouseEvent& event)
{
  Ogre::Ray ray;
  size_t goalSection = -1 ;
  size_t goalIndex = -1;
  Ogre::Real dist = -1;

  if ( singleRayQuery( event, num_results, ray) )
  {
    getIdentityOfSingleFace(reference_mesh, ray, goalSection, goalIndex, dist);

    if (goalIndex != -1)
    {

      std_msgs::Int32 index;
      index.data = goalIndex / 3;
      id_pub.publish(index);


      Ogre::Vector3 goal = ray.getPoint(dist);
      geometry_msgs::PoseStamped goal_msg;
      goal_msg.header.stamp = ros::Time::now();
      goal_msg.header.frame_id = context_->getFixedFrame().toStdString();
      goal_msg.pose.position.x = goal.x;
      goal_msg.pose.position.y = goal.y;
      goal_msg.pose.position.z = goal.z;

      goal_pub.publish(goal_msg);

      if (m_goalFaces.find( goalSection ) == m_goalFaces.end())
      {
        std::vector<size_t> faces;
        m_goalFaces.insert( std::pair<size_t, std::vector<size_t> >( goalSection, faces ) );
      }
      m_goalFaces[goalSection].push_back(goalIndex);
      std::sort( m_goalFaces[goalSection].begin(), m_goalFaces[goalSection].end() );
      m_goalFaces[goalSection].erase( std::unique( m_goalFaces[goalSection].begin(),
                                                   m_goalFaces[goalSection].end() ),
                                                   m_goalFaces[goalSection].end() );
      updateSelectionMesh();
    }
  }
}

void FaceSelectionTool::deselectSingleFace(rviz::ViewportMouseEvent& event)
{
  Ogre::Ray ray;
  size_t goalSection = -1 ;
  size_t goalIndex = -1;
  Ogre::Real dist = -1;

  if ( singleRayQuery( event, num_results, ray) )
  {
    getIdentityOfSingleFace(reference_mesh, ray, goalSection, goalIndex, dist);

    if (m_goalFaces.find(goalSection) != m_goalFaces.end())
    {
      if (std::find(m_goalFaces[goalSection].begin(),
                    m_goalFaces[goalSection].end(),
                    goalIndex) != m_goalFaces[goalSection].end())
      {
        m_goalFaces[goalSection].erase(std::find(m_goalFaces[goalSection].begin(),
                                                 m_goalFaces[goalSection].end(),
                                                 goalIndex));
        if (m_goalFaces[goalSection].size() < 1)
        {
            m_goalFaces.erase(goalSection);
        }
        updateSelectionMesh();
      }
    }
  }
}

void FaceSelectionTool::getIdentityOfSingleFace(Ogre::ManualObject* mesh,
                                        Ogre::Ray &ray,
                                        size_t &goalSection,
                                        size_t &goalIndex,
                                        Ogre::Real& closestDistance)
{
  closestDistance = -1.0f;
  size_t vertexCount = 0;
  Ogre::Vector3* vertices;
  size_t indexCount = 0;
  unsigned long* indices;
  size_t numSections = mesh->getNumSections();

  for (size_t i = 0; i < numSections; i++)
  {
    getRawManualObjectData(mesh, i, vertexCount, vertices, indexCount, indices);
    if(indexCount != 0)
    {
      for (size_t j = 0; j < indexCount; j += 3)
      {
        std::pair<bool, Ogre::Real> goal =
            Ogre::Math::intersects(
              ray,
              vertices[indices[j]],
              vertices[indices[j + 1]],
              vertices[indices[j + 2]],
              true,
              true);

        if (goal.first)
        {
          if ((closestDistance < 0.0f) || (goal.second < closestDistance))
          {
            closestDistance = goal.second;
            goalIndex = j;
            goalSection = i;
          }
        }
      }
    }

    delete[] vertices;
    delete[] indices;
  }
}

void FaceSelectionTool::getRawManualObjectData(Ogre::ManualObject *mesh,
                                       size_t sectionNumber,
                                       size_t &vertexCount,
                                       Ogre::Vector3* &vertices,
                                       size_t &indexCount,
                                       unsigned long* &indices)
{
  Ogre::VertexData* vertexData;
  const Ogre::VertexElement* vertexElement;
  Ogre::HardwareVertexBufferSharedPtr vertexBuffer;
  unsigned char* vertexChar;
  float* vertexFloat;

  vertexData = mesh->getSection(sectionNumber)->getRenderOperation()->vertexData;
  vertexElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
  vertexBuffer = vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());
  vertexChar = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

  vertexCount = vertexData->vertexCount;
  vertices = new Ogre::Vector3[vertexCount];

  for (size_t i = 0; i < vertexCount; i++, vertexChar += vertexBuffer->getVertexSize())
  {
      vertexElement->baseVertexPointerToElement(vertexChar, &vertexFloat);
      vertices[i] = (mesh->getParentNode()->_getDerivedOrientation() *
                    (Ogre::Vector3(vertexFloat[0], vertexFloat[1], vertexFloat[2]) * mesh->getParentNode()->_getDerivedScale())) +
                    mesh->getParentNode()->_getDerivedPosition();
  }

  vertexBuffer->unlock();

  Ogre::IndexData* indexData;
  Ogre::HardwareIndexBufferSharedPtr indexBuffer;
  indexData = mesh->getSection(sectionNumber)->getRenderOperation()->indexData;
  indexCount = indexData->indexCount;
  indices = new unsigned long[indexCount];
  indexBuffer = indexData->indexBuffer;
  unsigned int* pLong = static_cast<unsigned int*>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
  unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

  for (size_t i = 0; i < indexCount; i++)
  {
    unsigned long index;
    if (indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
    {
      index = static_cast<unsigned long>(pLong[i]);
    }

    else
    {
      index = static_cast<unsigned long>(pShort[i]);
    }

    indices[i] = index;
  }
  indexBuffer->unlock();
}

} // end namespace rviz_label_tool

