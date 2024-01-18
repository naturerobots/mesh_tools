cmake_minimum_required(VERSION 3.8)

project(mesh_msgs_hdf5)


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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(hdf5_map_io REQUIRED)
find_package(label_manager REQUIRED)

find_package(HDF5 REQUIRED COMPONENTS C CXX HL)

# LVR2 includes HighFive that we need here
# - it was compiling before because globale lvr2 header were accessible
# find_package(LVR2 REQUIRED)

include_directories(
  include
  ${HDF5_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/mesh_msgs_hdf5.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_action
  rclcpp_components
  actionlib_msgs
  mesh_msgs 
  hdf5_map_io
  label_manager)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
