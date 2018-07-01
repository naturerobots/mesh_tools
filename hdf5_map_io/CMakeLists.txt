cmake_minimum_required(VERSION 2.8.3)
project(hdf5_map_io)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hdf5_map_io
  DEPENDS HDF5
)

include_directories(
  include
  ext/HighFive/include
  ${HDF5_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hdf5_map_io.cpp
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

