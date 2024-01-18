cmake_minimum_required(VERSION 3.8)
project(rviz_mesh_tools_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hdf5_map_io REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
# find_package(assimp REQUIRED)

# This needs to be optional for non CL devices
# find_package(OpenCL 2 REQUIRED)

# include_directories(${ASSIMP_INCLUDE_DIRS})

# LVR2 includes HighFive that we need here
# - it was compiling before because globale lvr2 header were accessible
# find_package(LVR2 REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically.
# set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Core Widgets)
# add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${OpenCL_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/ClusterLabelDisplay.cpp
  src/ClusterLabelPanel.cpp
  src/ClusterLabelTool.cpp
  src/ClusterLabelVisual.cpp
  src/MapDisplay.cpp
  src/MeshDisplay.cpp
  src/MeshVisual.cpp
  src/RvizFileProperty.cpp
  src/MeshPoseTool.cpp
  src/MeshGoalTool.cpp
  src/MeshPoseGuessTool.cpp
  src/InteractionHelper.cpp
)


set(MOC_HEADER_FILES
  include/rviz_mesh_tools_plugins/ClusterLabelDisplay.hpp
  include/rviz_mesh_tools_plugins/ClusterLabelPanel.hpp
  include/rviz_mesh_tools_plugins/ClusterLabelVisual.hpp
  include/rviz_mesh_tools_plugins/MapDisplay.hpp
  include/rviz_mesh_tools_plugins/MeshDisplay.hpp
  include/rviz_mesh_tools_plugins/MeshVisual.hpp
  include/rviz_mesh_tools_plugins/ClusterLabelTool.hpp
  include/rviz_mesh_tools_plugins/CLUtil.hpp
  include/rviz_mesh_tools_plugins/RvizFileProperty.hpp
  include/rviz_mesh_tools_plugins/MeshPoseTool.hpp
  include/rviz_mesh_tools_plugins/MeshGoalTool.hpp
  include/rviz_mesh_tools_plugins/MeshPoseGuessTool.hpp
  include/rviz_mesh_tools_plugins/InteractionHelper.hpp
)

# foreach(header "${RVIZ_MESH_TOOLS_PLUGINS_MOC_FILES}")
#   qt5_wrap_cpp(RVIZ_MESH_TOOLS_PLUGINS_MOC "${header}")
# endforeach()



add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES} 
  ${MOC_HEADER_FILES} 
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  ${OpenCL_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Boost::system
  Boost::filesystem
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  ${OpenCL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_MESH_TOOLS_PLUGINS_BUILDING_LIBRARY")#

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rmw
  rviz_common
  rviz_rendering
  rviz_assimp_vendor
  std_msgs
  hdf5_map_io
  mesh_msgs
  resource_retriever
  tf2_ros
  message_filters
  std_msgs
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rmw
  rviz_common
  rviz_rendering
  rviz_assimp_vendor
  std_msgs
  hdf5_map_io
  mesh_msgs
  resource_retriever
  tf2_ros
  message_filters
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION share/${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "rviz_mesh_tools_plugins-extras.cmake"
)