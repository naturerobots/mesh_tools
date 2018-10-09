#ifndef MESH_MSGS_HDF5_H_
#define MESH_MSGS_HDF5_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <hdf5_map_io/hdf5_map_io.h>

#include <mesh_msgs/MeshFaceClusterStamped.h>
#include <mesh_msgs/GetGeometry.h>
#include <mesh_msgs/GetMaterials.h>
#include <mesh_msgs/GetTexture.h>
#include <mesh_msgs/GetUUID.h>
#include <mesh_msgs/GetVertexColors.h>
#include <mesh_msgs/GetVertexCosts.h>
#include <mesh_msgs/GetLabeledClusters.h>
#include <label_manager/GetLabelGroups.h>
#include <label_manager/GetLabeledClusterGroup.h>
#include <label_manager/DeleteLabel.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include <boost/algorithm/string.hpp>
#include <string>

namespace mesh_msgs_hdf5
{

class hdf5_to_msg
{

 public:
  hdf5_to_msg();

 protected:
  // Mesh services
  bool service_getGeometry(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);
  bool service_getGeometryVertices(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);
  bool service_getGeometryFaces(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);
  bool service_getGeometryVertexnormals(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);

  bool service_getMaterials(
      mesh_msgs::GetMaterials::Request &req,
      mesh_msgs::GetMaterials::Response &res);
  bool service_getTexture(
      mesh_msgs::GetTexture::Request &req,
      mesh_msgs::GetTexture::Response &res);
  bool service_getUUID(
      mesh_msgs::GetUUID::Request &req,
      mesh_msgs::GetUUID::Response &res);
  bool service_getVertexColors(
      mesh_msgs::GetVertexColors::Request &req,
      mesh_msgs::GetVertexColors::Response &res);

  // Label manager services
  bool service_getLabeledClusters(
      mesh_msgs::GetLabeledClusters::Request &req,
      mesh_msgs::GetLabeledClusters::Response &res);
  bool service_getLabelGroups(
      label_manager::GetLabelGroups::Request &req,
      label_manager::GetLabelGroups::Response &res);
  bool service_getLabeledClusterGroup(
      label_manager::GetLabeledClusterGroup::Request &req,
      label_manager::GetLabeledClusterGroup::Response &res);
  bool service_deleteLabel(
      label_manager::DeleteLabel::Request &req,
      label_manager::DeleteLabel::Response &res);

  // Vertex costs
  bool service_getRoughness(
      mesh_msgs::GetVertexCosts::Request &req,
      mesh_msgs::GetVertexCosts::Response &res);
  bool service_getHeightDifference(
      mesh_msgs::GetVertexCosts::Request &req,
      mesh_msgs::GetVertexCosts::Response &res);

  void callback_clusterLabel(const mesh_msgs::MeshFaceClusterStamped::ConstPtr &msg);

 private:

  // Mesh message service servers
  ros::ServiceServer srv_get_geometry_;
  ros::ServiceServer srv_get_geometry_vertices_;
  ros::ServiceServer srv_get_geometry_faces_;
  ros::ServiceServer srv_get_geometry_vertex_normals_;
  ros::ServiceServer srv_get_materials_;
  ros::ServiceServer srv_get_texture_;
  ros::ServiceServer srv_get_uuid_;
  ros::ServiceServer srv_get_vertex_colors_;
  ros::ServiceServer srv_get_roughness_;
  ros::ServiceServer srv_get_height_difference_;

  // Label manager services and subs/pubs
  ros::Subscriber sub_cluster_label_;
  ros::Publisher pub_cluster_label_;
  ros::ServiceServer srv_get_labeled_clusters_;
  ros::ServiceServer srv_get_label_groups_;
  ros::ServiceServer srv_get_labeled_cluster_group_;
  ros::ServiceServer srv_delete_label_;

  // ROS
  ros::NodeHandle node_handle;

  // ROS parameter
  std::string inputFile;

  std::string mesh_uuid = "mesh";

};

} // end namespace

#endif /* MESH_MSGS_HDF5_H_ */
