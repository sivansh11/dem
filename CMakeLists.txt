cmake_minimum_required(VERSION 3.10)

project(dem)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(FETCHCONTENT_QUIET FALSE)

include(FetchContent)

FetchContent_Declare(
  dawn
  GIT_REPOSITORY https://github.com/sivansh11/dawn
  GIT_TAG main
)
FetchContent_MakeAvailable(dawn)

message("-- fetching libfdt")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libfdt/)
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt.h)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt.h
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt.h"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_addresses.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_addresses.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_addresses.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_check.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_check.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_check.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_empty_tree.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_empty_tree.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_empty_tree.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_overlay.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_overlay.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_overlay.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_ro.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_ro.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_ro.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_rw.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_rw.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_rw.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_sterror.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_sterror.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_sterror.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_sw.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_sw.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_sw.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_wip.c)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/fdt_wip.c
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/fdt_wip.c"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/libfdt.h)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/libfdt.h
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/libfdt.h"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/libfdt_env.h)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/libfdt_env.h
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/libfdt_enf.h"
    STATUS DOWNLOAD_STATUS
  )
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libfdt/libfdt_internal.h)
  file(DOWNLOAD
    https://raw.githubusercontent.com/dgibson/dtc/main/libfdt/libfdt_internal.h
    "${CMAKE_CURRENT_BINARY_DIR}/libfdt/libfdt_internal.h"
    STATUS DOWNLOAD_STATUS
  )
endif()
message("-- fetched libfdt")

file(GLOB LIBFDT ${CMAKE_CURRENT_BINARY_DIR}/libfdt/*.c)

file(GLOB_RECURSE SRC_FILES src/*cpp)

add_executable(dem ${SRC_FILES} ${LIBFDT})
set_property(TARGET dem PROPERTY COMPILE_WARNING_AS_ERROR ON)

find_package(X11 REQUIRED)

target_link_libraries(dem
  PUBLIC dawn
  PUBLIC ${X11_LIBRARIES}
)

target_include_directories(dem
  PUBLIC src
  PUBLIC ${X11_INCLUDE_DIRS}
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/libfdt
)

target_include_directories(dem
  PUBLIC src
)

# default built type if CMAKE_BUILD_TYPE is not set
set(DEFAULT_BUILT_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILT_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILT_TYPE}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
