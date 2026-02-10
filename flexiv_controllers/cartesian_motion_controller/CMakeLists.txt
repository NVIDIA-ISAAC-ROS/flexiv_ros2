cmake_minimum_required(VERSION 3.8)
project(cartesian_motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(cartesian_motion_controller_parameters
  src/cartesian_motion_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_motion_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  cartesian_motion_controller_parameters
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  geometry_msgs
  std_msgs
  flexiv_msgs
)

pluginlib_export_plugin_description_file(controller_interface cartesian_motion_controller.xml)

install(
  TARGETS ${PROJECT_NAME} cartesian_motion_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  geometry_msgs
  std_msgs
  flexiv_msgs
)

ament_package()
