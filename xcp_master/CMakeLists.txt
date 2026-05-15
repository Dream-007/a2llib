cmake_minimum_required(VERSION 3.20)

project(xcp_master
        VERSION 1.0
        DESCRIPTION "XCP Master library and example (A2L-driven, CAN/CAN-FD transport)"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Locate liba2l (built by the parent a2llib project) so the example can
# parse A2L files and pull XCP-on-CAN IF_DATA.
# ---------------------------------------------------------------------------
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

option(XCP_MASTER_BUILD_EXAMPLE "Build the A2L-driven example" ON)
option(XCP_MASTER_STUB_DRIVER
       "Provide no-op stubs for the vendor CAN driver" ON)

# ---------------------------------------------------------------------------
# xcp_master library (transport-agnostic).
# ---------------------------------------------------------------------------
add_library(xcp_master STATIC
    src/xcp_master.cpp
    src/xcp_can_transport.cpp
    src/xcp_error.cpp
)

target_include_directories(xcp_master PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(xcp_master PUBLIC cxx_std_20)

if (MSVC)
    target_compile_options(xcp_master PRIVATE /W4)
else()
    target_compile_options(xcp_master PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Example (links liba2l + xcp_master).
# ---------------------------------------------------------------------------
if (XCP_MASTER_BUILD_EXAMPLE)
    if(NOT EXISTS "${A2L_STATIC_LIB}")
        message(WARNING
            "A2L_STATIC_LIB does not exist (${A2L_STATIC_LIB}); "
            "skipping example. Build liba2l first, then re-run cmake.")
    elseif(NOT EXISTS "${A2L_INCLUDE_DIR}/a2l/a2lfile.h")
        message(WARNING
            "A2L_INCLUDE_DIR is invalid (${A2L_INCLUDE_DIR}); "
            "skipping example.")
    else()
        if(EXISTS "${A2L_VCPKG_PREFIX}")
            list(PREPEND CMAKE_PREFIX_PATH "${A2L_VCPKG_PREFIX}")
        endif()

        find_package(Threads REQUIRED)
        find_package(Boost CONFIG REQUIRED COMPONENTS locale)

        add_library(a2l_static STATIC IMPORTED GLOBAL)
        set_target_properties(a2l_static PROPERTIES
            IMPORTED_LOCATION "${A2L_STATIC_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${A2L_INCLUDE_DIR}")

        add_executable(xcp_master_example
            example/main.cpp)

        if (XCP_MASTER_STUB_DRIVER)
            target_compile_definitions(xcp_master_example PRIVATE
                XCP_MASTER_STUB_DRIVER=1)
        endif()

        target_link_libraries(xcp_master_example PRIVATE
            xcp_master
            a2l_static
            Boost::locale
            Threads::Threads)
    endif()
endif()

# ---------------------------------------------------------------------------
# Install rules (header-only public API + static lib).
# ---------------------------------------------------------------------------
install(DIRECTORY include/xcp_master DESTINATION include)
install(TARGETS xcp_master
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
