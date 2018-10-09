cmake_minimum_required(VERSION 2.8.3)
project(hdf5_map_io)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)

catkin_package(
  INCLUDE_DIRS include ext/HighFive/include
  LIBRARIES hdf5_map_io
  DEPENDS HDF5
)

# HighFive
set(HIGHFIVE_EXAMPLES FALSE)
set(HIGHFIVE_UNIT_TESTS FALSE)
add_subdirectory(ext/HighFive)
link_directories(${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES})

include_directories(
  include
  ${HDF5_INCLUDE_DIRS}
  ext/HighFive/include
)

add_library(${PROJECT_NAME}
  src/hdf5_map_io.cpp
)

link_libraries(${PROJECT_NAME}
  ${HDF5_LIBRARIES}
  ${HDF5_C_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

