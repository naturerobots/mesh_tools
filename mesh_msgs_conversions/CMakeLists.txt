cmake_minimum_required(VERSION 3.1)

project(mesh_msgs_conversions)

set(PACKAGE_DEPENDENCIES
  mesh_msgs
  roscpp
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(LVR2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(MPI REQUIRED)
find_package(PkgConfig REQUIRED)

add_definitions(${LVR2_DEFINITIONS} ${OpenCV_DEFINITIONS})

### compile with c++14
set (CMAKE_CXX_STANDARD 14)

# enable openmp support
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LVR2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  INCLUDE_DIRS include
  DEPENDS LVR2 MPI
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/conversions.cpp
)

find_library(LVR2_LIBRARY NAMES lvr2)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LVR2_LIBRARY}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
