#include <rviz_mesh_tools_plugins/InteractionHelper.hpp>
#include <rviz_mesh_tools_plugins/MeshVisual.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>

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


bool selectFace(
  const MeshVisual& visual,
  const Ogre::Affine3& mesh_to_world,
  const Ogre::Ray& ray,
  Intersection& intersection)
{
  const Geometry& geometry = visual.getGeometry();

  // Transform the ray into the mesh frame instead of transforming every vertex into the world frame
  const Ogre::Affine3 world_to_mesh = mesh_to_world.inverse();
  // Directions are not affected by the translation, only by the linear part (rotation and scale)
  const Ogre::Ray mesh_ray(world_to_mesh * ray.getOrigin(), world_to_mesh.linear() * ray.getDirection());

  bool int_pos = true;
  bool int_neg = true;
  switch(visual.getCullingMode())
  {
    case Ogre::CULL_CLOCKWISE:
      int_pos = true;
      int_neg = false;
      break;
    case Ogre::CULL_ANTICLOCKWISE:
      int_pos = false;
      int_neg = true;
      break;
    case Ogre::CULL_NONE:
      int_pos = true;
      int_neg = true;
  }

  const auto vertex = [&geometry](uint32_t index) {
    const Vertex& v = geometry.vertices[index];
    return Ogre::Vector3(v.x, v.y, v.z);
  };

  Ogre::Real dist = -1.0f;

  for (size_t face_id = 0; face_id < geometry.faces.size(); face_id++)
  {
    const auto& indices = geometry.faces[face_id].vertexIndices;
    const Ogre::Vector3 v0 = vertex(indices[0]);
    const Ogre::Vector3 v1 = vertex(indices[1]);
    const Ogre::Vector3 v2 = vertex(indices[2]);

    std::pair<bool, Ogre::Real> goal = Ogre::Math::intersects(mesh_ray, v0, v1, v2, int_pos, int_neg);

    if (goal.first)
    {
      if ((dist < 0.0f) || (goal.second < dist))
      {
        dist = goal.second;
        intersection.face_id = face_id;
      }
    }
  }

  if (dist < 0.0f)
  {
    return false;
  }

  // Transform the hit back into the world frame. The range is measured along the world ray,
  // which differs from dist if the mesh frame is scaled.
  intersection.point = mesh_to_world * mesh_ray.getPoint(dist);
  intersection.range = (intersection.point - ray.getOrigin()).length() / ray.getDirection().length();
  const auto& indices = geometry.faces[intersection.face_id].vertexIndices;
  const Ogre::Vector3 a = mesh_to_world * vertex(indices[0]);
  Ogre::Vector3 ab = mesh_to_world * vertex(indices[1]) - a;
  Ogre::Vector3 ac = mesh_to_world * vertex(indices[2]) - a;
  intersection.normal = ac.crossProduct(ab).normalisedCopy();
  return true;
}

bool selectFace(
  Ogre::SceneManager* sm,
  const Ogre::Ray& ray, 
  Intersection& intersection)
{
  Ogre::RaySceneQuery* query = sm->createRayQuery(ray, Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
  query->setSortByDistance(true);

  // The result is owned by the query, so the query is destroyed after the loop
  Ogre::RaySceneQueryResult& result = query->execute();

  bool found = false;
  for (size_t i = 0; i < result.size(); i++)
  {
    // Only objects created by a MeshVisual carry a pointer back to it
    Ogre::Any binding = result[i].movable->getUserObjectBindings().getUserAny(MeshVisual::USER_ANY_KEY);
    MeshVisual** visual = Ogre::any_cast<MeshVisual*>(&binding);
    if (visual && *visual)
    {
      if (selectFace(**visual, result[i].movable->_getParentNodeFullTransform(), ray, intersection))
      {
        found = true;
        break;
      }
    }
  }

  sm->destroyQuery(query);
  return found;
}

bool selectFace(
  const rviz_common::DisplayContext* ctx,
  const Ogre::Ray& ray, 
  Intersection& intersection)
{
  return selectFace(ctx->getSceneManager(), ray, intersection);
}

} // namespace rviz_mesh_tools_plugins
