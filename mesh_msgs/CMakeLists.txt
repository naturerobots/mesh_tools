cmake_minimum_required(VERSION 2.8.3)
project(mesh_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  MeshFaceCluster.msg
  MeshFaceClusterStamped.msg
  Feature.msg
  MeshMaterial.msg
  MeshFeatures.msg
  MeshGeometry.msg
  MeshGeometryStamped.msg
  MeshMaterials.msg
  MeshMaterialsStamped.msg
  MeshVertexColors.msg
  MeshVertexColorsStamped.msg
  MeshVertexCosts.msg
  MeshVertexCostsStamped.msg
  MeshTexture.msg
  TriangleIndices.msg
  TriangleMesh.msg
  TriangleMeshStamped.msg
  VectorField.msg
  VectorFieldStamped.msg
  MeshVertexTexCoords.msg
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

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
)
