cmake_minimum_required(VERSION 3.8)
project(mesh_msgs_transform)

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
