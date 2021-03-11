cmake_minimum_required(VERSION 2.8.3)
project(rviz_map_plugin)

set(THIS_PACKAGE_ROS_DEPS
  roscpp
  rviz
  std_msgs
  mesh_msgs
  hdf5_map_io
)

find_package(catkin REQUIRED COMPONENTS
  ${THIS_PACKAGE_ROS_DEPS}
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
find_package(OpenCL 2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS Boost OpenCL HDF5 OpenCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${OpenCL_INCLUDE_DIRS}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

if("${rviz_QT_VERSION}" VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

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
  ${OpenCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
