cmake_minimum_required(VERSION 2.8.3)
project(rviz_mesh_plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


set(THIS_PACKAGE_ROS_DEPS
  roscpp
  rviz
  std_msgs
  mesh_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${THIS_PACKAGE_ROS_DEPS}
)

catkin_package(
  CATKIN_DEPENDS std_msgs mesh_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
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
  src/trianglemesh_display.cpp
  src/trianglemesh_visual.cpp
  src/textured_mesh_display.cpp
  src/textured_mesh_visual.cpp
  src/face_selection_tool.cpp
  src/mesh_pose_tool.cpp
  src/mesh_goal_tool.cpp
)

add_library(rviz_mesh_plugin ${SOURCE_FILES})
target_link_libraries(rviz_mesh_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(rviz_mesh_plugin ${catkin_EXPORTED_TARGETS})

install(TARGETS rviz_mesh_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
