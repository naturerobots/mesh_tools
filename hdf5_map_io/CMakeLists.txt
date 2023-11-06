cmake_minimum_required(VERSION 3.8)
project(hdf5_map_io)

set(CMAKE_CXX_STANDARD 14)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(LVR2 REQUIRED)
find_package(MPI)
find_package(PkgConfig REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES hdf5_map_io
#   DEPENDS LVR2 MPI
# )

# HighFive
set(HIGHFIVE_EXAMPLES FALSE)
set(HIGHFIVE_UNIT_TESTS FALSE)

include_directories(
  include
  ${LVR2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hdf5_map_io.cpp
)

find_library(LVR2_LIBRARY NAMES lvr2)

target_link_libraries(${PROJECT_NAME}
  ${LVR2_LIBRARY}
  ${MPI_CXX_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()