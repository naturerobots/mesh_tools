cmake_minimum_required(VERSION 3.0.2)
project(mesh_msgs_transform)

find_package(catkin REQUIRED COMPONENTS
    tf
    geometry_msgs
    mesh_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mesh_msgs_transform
  CATKIN_DEPENDS tf geometry_msgs mesh_msgs
)

add_library(mesh_msgs_transform
    src/transforms.cpp
)

target_link_libraries(mesh_msgs_transform
  ${catkin_LIBRARIES}
)

add_dependencies(mesh_msgs_transform ${catkin_EXPORTED_TARGETS})

install(TARGETS mesh_msgs_transform
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
