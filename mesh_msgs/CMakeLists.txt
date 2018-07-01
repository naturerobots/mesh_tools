cmake_minimum_required(VERSION 2.8.3)
project(mesh_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  Cluster.msg
  ClusterLabel.msg
  Feature.msg
  Material.msg
  MeshCostMap.msg
  MeshCostMapStamped.msg
  MeshFeatures.msg
  MeshGeometry.msg
  MeshGeometryStamped.msg
  MeshMaterials.msg
  MeshMaterialsStamped.msg
  MeshVertexColors.msg
  MeshVertexColorsStamped.msg
  MeshVertexCosts.msg
  MeshVertexCostsStamped.msg
  PolygonIndices.msg
  PolygonMesh.msg
  PolygonMeshStamped.msg
  Texture.msg
  TriangleIndices.msg
  TriangleMesh.msg
  TriangleMeshStamped.msg
  Vectorfield.msg
  VectorfieldStamped.msg
  VertexTexCoords.msg
)

add_service_files(
  DIRECTORY
  service
  FILES
  GetGeometry.srv
  GetLabeledClusters.srv
  GetMaterials.srv
  GetTexture.srv
  GetUUID.srv
  GetVertexColors.srv
  GetVertexCosts.srv
)

add_action_files(
  FILES
  CalcCosts.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
)
