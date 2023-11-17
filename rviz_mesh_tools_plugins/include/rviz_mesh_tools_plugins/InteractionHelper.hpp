#ifndef RVIZ_MESH_TOOLS_PLUGINS_MOUSE_EVENT_HELPER_HPP
#define RVIZ_MESH_TOOLS_PLUGINS_MOUSE_EVENT_HELPER_HPP

#include <OgreRay.h>
#include <rviz_common/viewport_mouse_event.hpp>


namespace rviz_common
{
  class DisplayContext;
} // namespace rviz_common

namespace rviz_mesh_tools_plugins
{

struct Intersection {
  double          range;
  Ogre::Vector3   point;
  Ogre::Vector3   normal;
  uint32_t        face_id;
  uint32_t        geom_id; // currently unused
  uint32_t        inst_id; // currently unused
};

Ogre::Ray getMouseEventRay(
  const rviz_common::ViewportMouseEvent& event);

bool selectFace(
  const Ogre::ManualObject* mesh, 
  const Ogre::Ray& ray,
  Intersection& intersection);

bool selectFace(
  const rviz_common::DisplayContext* ctx,
  const Ogre::Ray& ray,
  Intersection& intersection);

} // namespace rviz_mesh_tools_plugins


#endif // RVIZ_MESH_TOOLS_PLUGINS_MOUSE_EVENT_HELPER_HPP