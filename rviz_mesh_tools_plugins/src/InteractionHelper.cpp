#include <rviz_mesh_tools_plugins/InteractionHelper.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreSceneManager.h>

namespace rviz_mesh_tools_plugins
{

Ogre::Ray getMouseEventRay(
  const rviz_common::ViewportMouseEvent& event)
{
  auto viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();
  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
    static_cast<float>(event.x) / static_cast<float>(width),
    static_cast<float>(event.y) / static_cast<float>(height));
  return mouse_ray;
}


void getRawManualObjectData(
  const Ogre::ManualObject* mesh, 
  const size_t sectionNumber,
  size_t& vertexCount, Ogre::Vector3*& vertices, 
  size_t& indexCount, unsigned long*& indices)
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
    vertices[i] =
        (mesh->getParentNode()->_getDerivedOrientation() *
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

bool selectFace(
  const Ogre::ManualObject* mesh, 
  const Ogre::Ray& ray,
  Intersection& intersection)
{
  Ogre::Real dist = -1.0f;
  Ogre::Vector3 a, b, c;

  size_t vertex_count = 0;
  Ogre::Vector3* vertices;
  size_t index_count = 0;
  unsigned long* indices;
  size_t num_sections = mesh->getNumSections();

  for (size_t i = 0; i < num_sections; i++)
  {
    getRawManualObjectData(mesh, i, vertex_count, vertices, index_count, indices);
    if (index_count != 0)
    {
      for (size_t face_id = 0; face_id < index_count / 3; face_id++)
      {
        // face: indices[j], 
        const Ogre::Vector3 vertex;

        std::pair<bool, Ogre::Real> goal = Ogre::Math::intersects(ray, 
            vertices[indices[face_id * 3 + 0]], 
            vertices[indices[face_id * 3 + 1]],
            vertices[indices[face_id * 3 + 2]], true, true);

        if (goal.first)
        {
          if ((dist < 0.0f) || (goal.second < dist))
          {
            dist = goal.second;
            a = vertices[indices[face_id * 3 + 0]];
            b = vertices[indices[face_id * 3 + 1]];
            c = vertices[indices[face_id * 3 + 2]];
            intersection.face_id = face_id;
          }
        }
      }
    }
  }

  delete[] vertices;
  delete[] indices;
  if (dist != -1)
  {
    intersection.range = dist;
    intersection.point = ray.getPoint(dist);
    Ogre::Vector3 ab = b - a;
    Ogre::Vector3 ac = c - a;
    intersection.normal = ac.crossProduct(ab).normalisedCopy();
    return true;
  }
  else
  {
    return false;
  }
}

bool selectFace(
  Ogre::SceneManager* sm,
  const Ogre::Ray& ray, 
  Intersection& intersection)
{
  Ogre::RaySceneQuery* query = sm->createRayQuery(ray, Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
  query->setSortByDistance(true);

  Ogre::RaySceneQueryResult& result = query->execute();

  for (size_t i = 0; i < result.size(); i++)
  {
    if (result[i].movable->getName().find("TriangleMesh") != std::string::npos)
    {
      Ogre::ManualObject* mesh = static_cast<Ogre::ManualObject*>(result[i].movable);
      size_t goal_section = -1;
      size_t goal_index = -1;
      Ogre::Real dist = -1;
      if (selectFace(mesh, ray, intersection))
      {
        return true;
      }
    }
  }
  return false;
}

bool selectFace(
  const rviz_common::DisplayContext* ctx,
  const Ogre::Ray& ray, 
  Intersection& intersection)
{
  return selectFace(ctx->getSceneManager(), ray, intersection);
}

} // namespace rviz_mesh_tools_plugins