cmake_minimum_required(VERSION 3.8)
project(label_manager)


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
find_package(rosidl_default_generators REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost COMPONENTS 
  system 
  filesystem
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DeleteLabel.srv"
  "srv/GetLabelGroups.srv"
  "srv/GetLabeledClusterGroup.srv"
  DEPENDENCIES
    mesh_msgs
    std_msgs
  ADD_LINTER_TESTS
)


include_directories(
  include
)

add_executable(${PROJECT_NAME}_node
  src/manager.cpp
  src/manager_node.cpp)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
  Boost::system 
  Boost::filesystem
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  rclcpp_components
  mesh_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node 
  "${cpp_typesupport_target}")


ament_export_dependencies(rosidl_default_runtime)
ament_package()

