cmake_minimum_required(VERSION 3.0.2)
project(label_manager)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  mesh_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf
)


add_service_files(FILES
  DeleteLabel.srv
  GetLabelGroups.srv
  GetLabeledClusterGroup.srv
)

generate_messages(DEPENDENCIES
  mesh_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib actionlib_msgs genmsg mesh_msgs message_generation message_runtime roscpp sensor_msgs std_msgs tf
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}
  src/manager.cpp
  src/manager_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

