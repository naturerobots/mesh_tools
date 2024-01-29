cmake_minimum_required(VERSION 3.8)
project(mesh_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MeshFaceCluster.msg"
  "msg/MeshFaceClusterStamped.msg"
  "msg/MeshMaterial.msg"
  "msg/MeshGeometry.msg"
  "msg/MeshGeometryStamped.msg"
  "msg/MeshMaterials.msg"
  "msg/MeshMaterialsStamped.msg"
  "msg/MeshVertexColors.msg"
  "msg/MeshVertexColorsStamped.msg"
  "msg/MeshVertexCosts.msg"
  "msg/MeshVertexCostsStamped.msg"
  "msg/MeshTexture.msg"
  "msg/MeshTriangleIndices.msg"
  "msg/VectorField.msg"
  "msg/VectorFieldStamped.msg"
  "msg/MeshVertexTexCoords.msg"
  "srv/GetGeometry.srv"
  "srv/GetLabeledClusters.srv"
  "srv/GetMaterials.srv"
  "srv/GetTexture.srv"
  "srv/GetUUIDs.srv"
  "srv/GetVertexColors.srv"
  "srv/GetVertexCosts.srv"
  "srv/GetVertexCostLayers.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()