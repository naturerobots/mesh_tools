cmake_minimum_required(VERSION 3.8)
project(mesh_msgs_conversions)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default: Release
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(LVR2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(MPI REQUIRED)
find_package(PkgConfig REQUIRED)

add_definitions(${LVR2_DEFINITIONS} ${OpenCV_DEFINITIONS})

# enable openmp support
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
include_directories(
  include
  ${LVR2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/conversions.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${LVR2_LIBRARY}
  ${OpenCV_LIBRARIES}
  rclcpp::rclcpp
  ${mesh_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)


target_compile_definitions(${PROJECT_NAME} PRIVATE "MESH_MSGS_CONVERSIONS_BUILDING_DLL")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  mesh_msgs
  sensor_msgs
)

ament_package()