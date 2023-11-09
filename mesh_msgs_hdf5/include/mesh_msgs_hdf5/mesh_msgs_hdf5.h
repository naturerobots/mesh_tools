#ifndef MESH_MSGS_HDF5_H_
#define MESH_MSGS_HDF5_H_


#include <boost/algorithm/string.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>

#include <hdf5_map_io/hdf5_map_io.h>

#include "mesh_msgs/msg/mesh_face_cluster_stamped.hpp"
#include "mesh_msgs/srv/get_geometry.hpp"
#include "mesh_msgs/srv/get_materials.hpp"
#include "mesh_msgs/srv/get_texture.hpp"
#include "mesh_msgs/srv/get_uui_ds.hpp" // :D GetUUIDs
#include "mesh_msgs/srv/get_vertex_colors.hpp"
#include "mesh_msgs/srv/get_vertex_costs.hpp"
#include "mesh_msgs/srv/get_vertex_cost_layers.hpp"
#include "mesh_msgs/srv/get_labeled_clusters.hpp"
#include "label_manager/srv/get_label_groups.hpp"
#include "label_manager/srv/get_labeled_cluster_group.hpp"
#include "label_manager/srv/delete_label.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/fill_image.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mesh_msgs_hdf5
{

class hdf5_to_msg : public rclcpp::Node
{

 public:
  hdf5_to_msg();

 protected:
  void loadAndPublishGeometry();

  bool getVertices(std::vector<float>& vertices, mesh_msgs::msg::MeshGeometryStamped& geometryMsg);
  bool getFaces(std::vector<uint32_t>& faceIds, mesh_msgs::msg::MeshGeometryStamped& geometryMsg);
  bool getVertexNormals(std::vector<float>& vertexNormals, mesh_msgs::msg::MeshGeometryStamped& geometryMsg);

  bool getVertexColors(std::vector<uint8_t>& vertexColors, mesh_msgs::msg::MeshVertexColorsStamped& vertexColorsMsg);
  bool getVertexCosts(std::vector<float>& vertexCosts, std::string layer, mesh_msgs::msg::MeshVertexCostsStamped& vertexCostsMsg);

  // Mesh services
  bool service_getUUIDs(
      const std::shared_ptr<mesh_msgs::srv::GetUUIDs::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetUUIDs::Response> res);
  bool service_getGeometry(
      const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res);
  bool service_getGeometryVertices(
      const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res);
  bool service_getGeometryFaces(
      const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res);
  bool service_getGeometryVertexNormals(
      const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res);

  bool service_getMaterials(
      const std::shared_ptr<mesh_msgs::srv::GetMaterials::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetMaterials::Response> res);
  bool service_getTexture(
      const std::shared_ptr<mesh_msgs::srv::GetTexture::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetTexture::Response> res);
  bool service_getVertexColors(
      const std::shared_ptr<mesh_msgs::srv::GetVertexColors::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetVertexColors::Response> res);
  bool service_getVertexCosts(
      const std::shared_ptr<mesh_msgs::srv::GetVertexCosts::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetVertexCosts::Response> res);
  bool service_getVertexCostLayers(
      const std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers::Response> res);

  // Label manager services
  bool service_getLabeledClusters(
      const std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Request> req,
      std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Response> res);

  void callback_clusterLabel(const mesh_msgs::msg::MeshFaceClusterStamped& msg);

 private:

  // Mesh message service servers

  rclcpp::Service<mesh_msgs::srv::GetUUIDs>::SharedPtr srv_get_uuids_;
  rclcpp::Service<mesh_msgs::srv::GetGeometry>::SharedPtr srv_get_geometry_;
  rclcpp::Service<mesh_msgs::srv::GetGeometry>::SharedPtr srv_get_geometry_vertices_;
  rclcpp::Service<mesh_msgs::srv::GetGeometry>::SharedPtr srv_get_geometry_faces_;
  rclcpp::Service<mesh_msgs::srv::GetGeometry>::SharedPtr srv_get_geometry_vertex_normals_;
  rclcpp::Service<mesh_msgs::srv::GetMaterials>::SharedPtr srv_get_materials_;
  rclcpp::Service<mesh_msgs::srv::GetTexture>::SharedPtr srv_get_texture_;
  rclcpp::Service<mesh_msgs::srv::GetVertexColors>::SharedPtr srv_get_vertex_colors_;
  rclcpp::Service<mesh_msgs::srv::GetVertexCosts>::SharedPtr srv_get_vertex_costs_;
  rclcpp::Service<mesh_msgs::srv::GetVertexCostLayers>::SharedPtr srv_get_vertex_cost_layers_;

  // Mesh message publishers
  rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr pub_geometry_;
  rclcpp::Publisher<mesh_msgs::msg::MeshVertexColorsStamped>::SharedPtr pub_vertex_colors_;
  rclcpp::Publisher<mesh_msgs::msg::MeshVertexCostsStamped>::SharedPtr pub_vertex_costs_;

  // Label manager services and subs/pubs
  rclcpp::Service<mesh_msgs::srv::GetLabeledClusters>::SharedPtr srv_get_labeled_clusters_;
  rclcpp::Subscription<mesh_msgs::msg::MeshFaceClusterStamped>::SharedPtr sub_cluster_label_;

  // ROS parameter
  std::string inputFile;

  std::string mesh_uuid = "mesh";
};

} // end namespace

#endif /* MESH_MSGS_HDF5_H_ */
