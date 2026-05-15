cmake_minimum_required(VERSION 3.20)

project(a2l_json_exporter
        VERSION 1.0
        DESCRIPTION "Export A2L measurements and conversions to JSON"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(A2LLIB_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(A2L_STATIC_LIB
    "${A2LLIB_ROOT}/build-local-vcpkg/liba2l.a"
    CACHE FILEPATH "Path to the built liba2l static library")
set(A2L_INCLUDE_DIR
    "${A2LLIB_ROOT}/include"
    CACHE PATH "Path to a2llib public headers")
set(A2L_VCPKG_PREFIX
    "${A2LLIB_ROOT}/build/vcpkg_installed/x64-linux"
    CACHE PATH "Prefix containing Boost built for liba2l")

if(NOT EXISTS "${A2L_STATIC_LIB}")
  message(FATAL_ERROR "A2L_STATIC_LIB does not exist: ${A2L_STATIC_LIB}")
endif()

if(NOT EXISTS "${A2L_INCLUDE_DIR}/a2l/a2lfile.h")
  message(FATAL_ERROR "A2L_INCLUDE_DIR is invalid: ${A2L_INCLUDE_DIR}")
endif()

if(EXISTS "${A2L_VCPKG_PREFIX}")
  list(PREPEND CMAKE_PREFIX_PATH "${A2L_VCPKG_PREFIX}")
endif()

find_package(Threads REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS locale)

add_library(a2l_static STATIC IMPORTED GLOBAL)
set_target_properties(a2l_static PROPERTIES
    IMPORTED_LOCATION "${A2L_STATIC_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${A2L_INCLUDE_DIR}")

add_executable(a2l_json_exporter
    src/main.cpp)

target_link_libraries(a2l_json_exporter PRIVATE
    a2l_static
    Boost::locale
    Threads::Threads)
