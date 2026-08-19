cmake_minimum_required(VERSION 3.5)
project(flexiv_gripper)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(flexiv_hardware REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Use the symbol-isolated RDK wrapper on Lyrical. Jazzy continues to consume
# the vendor's regular CMake package.
set(FLEXIV_RDK_PREFIX "$ENV{FLEXIV_RDK_PREFIX}" CACHE PATH
  "Private flexiv_rdk installation prefix")
if("$ENV{ROS_DISTRO}" STREQUAL "lyrical")
  set(_flexiv_rdk_wrapper
    "${FLEXIV_RDK_PREFIX}/lib/libflexiv_rdk_lyrical.so")
  if(NOT FLEXIV_RDK_PREFIX OR NOT EXISTS "${_flexiv_rdk_wrapper}")
    message(FATAL_ERROR
      "ROS 2 Lyrical requires the symbol-isolated RDK wrapper. Run "
      "scripts/build_and_install_rdk.sh and export FLEXIV_RDK_PREFIX.")
  endif()
  add_library(flexiv_rdk_lyrical SHARED IMPORTED)
  set_target_properties(flexiv_rdk_lyrical PROPERTIES
    IMPORTED_LOCATION "${_flexiv_rdk_wrapper}")
  set(FLEXIV_RDK_TARGET flexiv_rdk_lyrical)
else()
  set(_flexiv_original_cmake_prefix_path "${CMAKE_PREFIX_PATH}")
  if(FLEXIV_RDK_PREFIX)
    list(PREPEND CMAKE_PREFIX_PATH "${FLEXIV_RDK_PREFIX}")
  endif()
  find_package(flexiv_rdk REQUIRED)
  set(CMAKE_PREFIX_PATH "${_flexiv_original_cmake_prefix_path}")
  unset(_flexiv_original_cmake_prefix_path)
  set(FLEXIV_RDK_TARGET flexiv::flexiv_rdk)
endif()

add_library(gripper_action_server
  SHARED
  src/gripper_action_server.cpp)

target_include_directories(gripper_action_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_hardware/rdk/include)

target_link_libraries(gripper_action_server
  ${FLEXIV_RDK_TARGET}
)

ament_target_dependencies(gripper_action_server
  control_msgs
  flexiv_hardware
  flexiv_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_srvs)

rclcpp_components_register_node(gripper_action_server
  PLUGIN "flexiv_gripper::GripperActionServer"
  EXECUTABLE flexiv_gripper_node)

install(TARGETS gripper_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
