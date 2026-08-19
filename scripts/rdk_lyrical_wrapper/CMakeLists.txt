cmake_minimum_required(VERSION 3.24)
project(flexiv_rdk_lyrical VERSION 1.9.2 LANGUAGES CXX)

if(NOT FLEXIV_RDK_PRIVATE_PREFIX)
  message(FATAL_ERROR "FLEXIV_RDK_PRIVATE_PREFIX must point to the static RDK installation")
endif()

list(PREPEND CMAKE_PREFIX_PATH "${FLEXIV_RDK_PRIVATE_PREFIX}")

# Make dependency configs select their static exports when both variants are
# present. This configure runs outside the ROS workspace, so target names such
# as fastcdr cannot collide with Lyrical's targets.
set(BUILD_SHARED_LIBS OFF)
set(fastcdr_SHARED_LIBS OFF)
set(fastrtps_SHARED_LIBS OFF)
find_package(flexiv_rdk 1.9.2 CONFIG REQUIRED)

add_library(flexiv_rdk_lyrical SHARED wrapper_anchor.cpp)

# Pull every object from the proprietary RDK archive into this DSO. Its static
# dependency targets are linked normally and only the referenced objects are
# retained.
target_link_libraries(flexiv_rdk_lyrical
  PRIVATE
    "$<LINK_LIBRARY:WHOLE_ARCHIVE,flexiv::flexiv_rdk>"
)

target_link_options(flexiv_rdk_lyrical
  PRIVATE
    "LINKER:-Bsymbolic"
    "LINKER:-z,defs"
    "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/flexiv_rdk_lyrical.map"
)

set_target_properties(flexiv_rdk_lyrical PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  OUTPUT_NAME flexiv_rdk_lyrical
  VERSION 1.9.2
  SOVERSION 1
)

include(GNUInstallDirs)
install(TARGETS flexiv_rdk_lyrical
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
