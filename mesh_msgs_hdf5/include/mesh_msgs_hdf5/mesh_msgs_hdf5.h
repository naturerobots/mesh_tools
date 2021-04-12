#ifndef MESH_MSGS_HDF5_H_
#define MESH_MSGS_HDF5_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <hdf5_map_io/hdf5_map_io.h>

#include <mesh_msgs/MeshFaceClusterStamped.h>
#include <mesh_msgs/GetGeometry.h>
#include <mesh_msgs/GetMaterials.h>
#include <mesh_msgs/GetTexture.h>
#include <mesh_msgs/GetUUIDs.h>
#include <mesh_msgs/GetVertexColors.h>
#include <mesh_msgs/GetVertexCosts.h>
#include <mesh_msgs/GetVertexCostLayers.h>
#include <mesh_msgs/GetLabeledClusters.h>
#include <label_manager/GetLabelGroups.h>
#include <label_manager/GetLabeledClusterGroup.h>
#include <label_manager/DeleteLabel.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include <boost/algorithm/string.hpp>
#include <string>
#include <vector>

namespace mesh_msgs_hdf5
{

class hdf5_to_msg
{

 public:
  hdf5_to_msg();

 protected:
  void loadAndPublishGeometry();

  bool getVertices(std::vector<float>& vertices, mesh_msgs::MeshGeometryStamped& geometryMsg);
  bool getFaces(std::vector<uint32_t>& faceIds, mesh_msgs::MeshGeometryStamped& geometryMsg);
  bool getVertexNormals(std::vector<float>& vertexNormals, mesh_msgs::MeshGeometryStamped& geometryMsg);

  bool getVertexColors(std::vector<uint8_t>& vertexColors, mesh_msgs::MeshVertexColorsStamped& vertexColorsMsg);
  bool getVertexCosts(std::vector<float>& vertexCosts, std::string layer, mesh_msgs::MeshVertexCostsStamped& vertexCostsMsg);

  // Mesh services
  bool service_getUUIDs(
      mesh_msgs::GetUUIDs::Request &req,
      mesh_msgs::GetUUIDs::Response &res);
  bool service_getGeometry(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);
  bool service_getGeometryVertices(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);
  bool service_getGeometryFaces(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);
  bool service_getGeometryVertexNormals(
      mesh_msgs::GetGeometry::Request &req,
      mesh_msgs::GetGeometry::Response &res);

  bool service_getMaterials(
      mesh_msgs::GetMaterials::Request &req,
      mesh_msgs::GetMaterials::Response &res);
  bool service_getTexture(
      mesh_msgs::GetTexture::Request &req,
      mesh_msgs::GetTexture::Response &res);
  bool service_getVertexColors(
      mesh_msgs::GetVertexColors::Request &req,
      mesh_msgs::GetVertexColors::Response &res);
  bool service_getVertexCosts(
      mesh_msgs::GetVertexCosts::Request &req,
      mesh_msgs::GetVertexCosts::Response &res);
  bool service_getVertexCostLayers(
      mesh_msgs::GetVertexCostLayers::Request &req,
      mesh_msgs::GetVertexCostLayers::Response &res);

  // Label manager services
  bool service_getLabeledClusters(
      mesh_msgs::GetLabeledClusters::Request &req,
      mesh_msgs::GetLabeledClusters::Response &res);

  void callback_clusterLabel(const mesh_msgs::MeshFaceClusterStamped::ConstPtr &msg);

 private:

  // Mesh message service servers
  ros::ServiceServer srv_get_uuids_;
  ros::ServiceServer srv_get_geometry_;
  ros::ServiceServer srv_get_geometry_vertices_;
  ros::ServiceServer srv_get_geometry_faces_;
  ros::ServiceServer srv_get_geometry_vertex_normals_;
  ros::ServiceServer srv_get_materials_;
  ros::ServiceServer srv_get_texture_;
  ros::ServiceServer srv_get_vertex_colors_;
  ros::ServiceServer srv_get_vertex_costs_;
  ros::ServiceServer srv_get_vertex_cost_layers_;

  // Mesh message publishers
  ros::Publisher pub_geometry_;
  ros::Publisher pub_vertex_colors_;
  ros::Publisher pub_vertex_costs_;

  // Label manager services and subs/pubs
  ros::ServiceServer srv_get_labeled_clusters_;
  ros::Subscriber sub_cluster_label_;

  // ROS
  ros::NodeHandle node_handle;

  // ROS parameter
  std::string inputFile;

  std::string mesh_uuid = "mesh";

};

} // end namespace

#endif /* MESH_MSGS_HDF5_H_ */
