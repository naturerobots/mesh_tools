cmake_minimum_required(VERSION 3.1)

project(mesh_msgs_hdf5)

set(PACKAGE_DEPENDENCIES
  mesh_msgs
  hdf5_map_io
  label_manager
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)

### compile with c++14
set(CMAKE_CXX_STANDARD 14)

include_directories(
  include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  INCLUDE_DIRS include
  DEPENDS HDF5
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/mesh_msgs_hdf5.cpp
)

target_link_libraries(${PROJECT_NAME} 
    ${HDF5_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#add_executable(${PROJECT_NAME}_node src/hdf5_to_msg_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#  ${HDF5_LIBRARIES}
#)

#install(
#  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(
  TARGETS ${PROJECT_NAME}# ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
