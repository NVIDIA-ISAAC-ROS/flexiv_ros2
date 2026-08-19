cmake_minimum_required(VERSION 3.5)
project(flexiv_hardware)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(flexiv_msgs REQUIRED)

# Lyrical cannot load the RDK's Fast-CDR 1 and ROS's Fast-CDR 2 symbols into
# the same ELF namespace. The helper installs an RDK wrapper whose private
# dependencies are statically embedded and symbol-hidden.
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

add_library(
  ${PROJECT_NAME}
  SHARED
  src/flexiv_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rdk/include
)

target_link_libraries(${PROJECT_NAME}
  ${FLEXIV_RDK_TARGET}
)

# Link ROS packages
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  flexiv_msgs
)

pluginlib_export_plugin_description_file(hardware_interface flexiv_hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS scripts/export_calibrated_model.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_pytest_test(
    test_export_calibrated_model
    test/test_export_calibrated_model.py
  )
endif()

# EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  flexiv_msgs
)

ament_package()
