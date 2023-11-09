cmake_minimum_required(VERSION 3.8)
project(rviz_map_plugin)

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
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hdf5_map_io REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)


find_package(Boost REQUIRED COMPONENTS system)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
find_package(OpenCL 2)


find_package(assimp)

if(assimp_FOUND)
  include_directories(${ASSIMP_INCLUDE_DIRS})
  add_definitions(-DWITH_ASSIMP)
endif(assimp_FOUND)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${OpenCL_INCLUDE_DIRS}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)


add_definitions(-DQT_NO_KEYWORDS)

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
)

set(HEADER_FILES
  include/ClusterLabelDisplay.hpp
  include/ClusterLabelPanel.hpp
  include/ClusterLabelVisual.hpp
  include/MapDisplay.hpp
  include/MeshDisplay.hpp
  include/MeshVisual.hpp
  include/ClusterLabelTool.hpp
  include/CLUtil.hpp
  include/RvizFileProperty.hpp
  include/MeshPoseTool.hpp
  include/MeshGoalTool.hpp
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES} ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES})

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  std_msgs
  mesh_msgs
  rviz_common
  rviz_rendering
  tf2_ros
  pluginlib
  message_filters
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)
install(FILES
  rviz_plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

ament_package()