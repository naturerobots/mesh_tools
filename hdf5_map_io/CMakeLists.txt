cmake_minimum_required(VERSION 3.8)
project(hdf5_map_io)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(LVR2 REQUIRED)
find_package(MPI)
find_package(PkgConfig REQUIRED)


# HighFive
set(HIGHFIVE_EXAMPLES FALSE)
set(HIGHFIVE_UNIT_TESTS FALSE)

find_package(LVR2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/hdf5_map_io.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${LVR2_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  ${LVR2_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
  rclcpp::rclcpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "HDF5_MAP_IO_BUILDING_DLL")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  LVR2
)

ament_package()