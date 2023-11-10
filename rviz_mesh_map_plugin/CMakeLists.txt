cmake_minimum_required(VERSION 3.8)
project(rviz_mesh_map_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hdf5_map_io REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
find_package(assimp REQUIRED)

# This needs to be optional for non CL devices
find_package(OpenCL 2)

include_directories(${ASSIMP_INCLUDE_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
# set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Core Widgets)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${OpenCL_INCLUDE_DIRS}
)

set(SOURCE_FILES
  # src/ClusterLabelDisplay.cpp
  # src/ClusterLabelPanel.cpp
  # src/ClusterLabelTool.cpp
  # src/ClusterLabelVisual.cpp
  # src/MapDisplay.cpp
  src/MeshDisplay.cpp
  src/MeshVisual.cpp
  # src/RvizFileProperty.cpp
  # src/MeshPoseTool.cpp
  # src/MeshGoalTool.cpp
)


set(MOC_HEADER_FILES
  # include/ClusterLabelDisplay.hpp
  # include/ClusterLabelPanel.hpp
  # include/ClusterLabelVisual.hpp
  # include/MapDisplay.hpp
  include/MeshDisplay.hpp
  include/MeshVisual.hpp
  # include/ClusterLabelTool.hpp
  # include/CLUtil.hpp
  # include/RvizFileProperty.hpp
  # include/MeshPoseTool.hpp
  # include/MeshGoalTool.hpp
)

foreach(header "${RVIZ_MESH_MAP_PLUGIN_MOC_FILES}")
  qt5_wrap_cpp(RVIZ_MESH_MAP_PLUGIN_MOC_FILES "${header}")
endforeach()

add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES} 
  ${RVIZ_MESH_MAP_PLUGIN_MOC_FILES} 
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_MESH_MAP_PLUGIN_BUILDING_LIBRARY")#

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rmw
  rviz_common
  rviz_rendering
  std_msgs
  hdf5_map_io
  mesh_msgs
  resource_retriever
  tf2_ros
  message_filters
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)



# install(FILES
#   plugins_description.xml
#   DESTINATION share/${PROJECT_NAME})


install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

ament_package()