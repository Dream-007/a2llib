cmake_minimum_required(VERSION 3.20)

project(xcp_master
        VERSION 1.0
        DESCRIPTION "XCP Master library and example (A2L-driven, TOSUN TSCAN CAN-FD transport)"
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

# ---------------------------------------------------------------------------
# Locate the TOSUN TSCAN (TSMaster) SDK.
# ---------------------------------------------------------------------------
set(TSCAN_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    CACHE PATH "Directory containing libTSCANApiOnLinux.so and libTSH.so")
set(TSCAN_API_LIB "${TSCAN_LIB_DIR}/libTSCANApiOnLinux.so"
    CACHE FILEPATH "Path to libTSCANApiOnLinux.so")
set(TSCAN_HELPER_LIB "${TSCAN_LIB_DIR}/libTSH.so"
    CACHE FILEPATH "Path to libTSH.so (dlopen'd from cwd by the API lib)")

option(XCP_MASTER_BUILD_EXAMPLE "Build the A2L-driven example" ON)

# ---------------------------------------------------------------------------
# Optional integration with liba2l: when the a2l prerequisites are available
# we compile signal_access.cpp into the library and PUBLIC-expose the
# `XcpSignalAccess` symbol-name-driven read/write helper.  When they are
# not, the rest of xcp_master still builds.
# ---------------------------------------------------------------------------
set(_xcp_master_have_a2l FALSE)
if(EXISTS "${A2L_STATIC_LIB}" AND EXISTS "${A2L_INCLUDE_DIR}/a2l/a2lfile.h")
    if(EXISTS "${A2L_VCPKG_PREFIX}")
        list(PREPEND CMAKE_PREFIX_PATH "${A2L_VCPKG_PREFIX}")
    endif()
    find_package(Boost CONFIG QUIET COMPONENTS locale)
    if(Boost_FOUND)
        add_library(a2l_static STATIC IMPORTED GLOBAL)
        set_target_properties(a2l_static PROPERTIES
            IMPORTED_LOCATION "${A2L_STATIC_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${A2L_INCLUDE_DIR}")
        set(_xcp_master_have_a2l TRUE)
    else()
        message(STATUS
            "xcp_master: Boost::locale not found - signal_access disabled")
    endif()
else()
    message(STATUS
        "xcp_master: liba2l not built yet - signal_access disabled "
        "(re-run cmake after building liba2l to enable it)")
endif()

# ---------------------------------------------------------------------------
# xcp_master library (transport + master + SDK device wrapper).
# ---------------------------------------------------------------------------
set(_xcp_master_sources
    src/xcp_master.cpp
    src/xcp_can_transport.cpp
    src/xcp_error.cpp
    src/ts_can_device.cpp
)
if(_xcp_master_have_a2l)
    list(APPEND _xcp_master_sources src/signal_access.cpp)
endif()

add_library(xcp_master STATIC ${_xcp_master_sources})

if(_xcp_master_have_a2l)
    target_link_libraries(xcp_master PUBLIC a2l_static Boost::locale)
endif()

target_include_directories(xcp_master PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(xcp_master PUBLIC cxx_std_20)

# The static library calls into the SDK directly; the example pulls the .so
# in via PRIVATE link below.  We mark the SDK as an INTERFACE dependency so
# anything that links against xcp_master also picks it up.
#
# IMPORTED_NO_SONAME=TRUE is required because libTSCANApiOnLinux.so does not
# carry a DT_SONAME; without this flag the linker embeds the build-time
# absolute path of the .so as DT_NEEDED, which defeats the $ORIGIN rpath we
# set on the executable and breaks relocation of the build tree.
add_library(tscan_api SHARED IMPORTED GLOBAL)
set_target_properties(tscan_api PROPERTIES
    IMPORTED_LOCATION "${TSCAN_API_LIB}"
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(xcp_master PUBLIC tscan_api)

if (MSVC)
    target_compile_options(xcp_master PRIVATE /W4)
else()
    target_compile_options(xcp_master PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Example (links liba2l + xcp_master + the TOSUN SDK).
# ---------------------------------------------------------------------------
if (XCP_MASTER_BUILD_EXAMPLE)
    if(NOT _xcp_master_have_a2l)
        message(WARNING
            "xcp_master example needs liba2l + Boost::locale (see status "
            "messages above); skipping example.")
    elseif(NOT EXISTS "${TSCAN_API_LIB}")
        message(WARNING
            "TSCAN_API_LIB does not exist (${TSCAN_API_LIB}); "
            "skipping example.")
    else()
        find_package(Threads REQUIRED)

        add_executable(xcp_master_example
            example/main.cpp)

        target_link_libraries(xcp_master_example PRIVATE
            xcp_master
            Threads::Threads
            ${CMAKE_DL_LIBS})

        # ---- runtime layout --------------------------------------------------
        # The TOSUN SDK is shipped as a pair of .so's:
        #   libTSCANApiOnLinux.so  - we link against it directly,
        #   libTSH.so              - dlopen'd by the API .so at runtime
        #                            *relative to the current working directory*.
        #
        # If libTSH.so cannot be located when tscan_transmit_canfd_async /
        # tsfifo_receive_canfd_msgs run, the SDK returns error code 100.
        # The reliable fix is to:
        #   1. set RPATH=$ORIGIN so libTSCANApiOnLinux.so is found next to
        #      the executable when it is launched from anywhere;
        #   2. copy libTSH.so into the same directory as the executable so
        #      the dlopen("libTSH.so") inside the SDK succeeds via the
        #      default loader search path (the working directory typically
        #      ends up here when the user runs `./xcp_master_example`).
        set_target_properties(xcp_master_example PROPERTIES
            BUILD_RPATH   "$ORIGIN"
            INSTALL_RPATH "$ORIGIN"
            BUILD_WITH_INSTALL_RPATH TRUE)

        # Stage both .so's next to the executable so `./xcp_master_example`
        # from the build dir works without LD_LIBRARY_PATH games.
        add_custom_command(TARGET xcp_master_example POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${TSCAN_API_LIB}"
                "$<TARGET_FILE_DIR:xcp_master_example>/libTSCANApiOnLinux.so"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${TSCAN_HELPER_LIB}"
                "$<TARGET_FILE_DIR:xcp_master_example>/libTSH.so"
            COMMENT "Staging libTSCANApiOnLinux.so + libTSH.so next to the example")
    endif()
endif()

# ---------------------------------------------------------------------------
# Install rules (header-only public API + static lib + runtime SDK pair).
# ---------------------------------------------------------------------------
install(DIRECTORY include/xcp_master DESTINATION include)
install(TARGETS xcp_master
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
if(EXISTS "${TSCAN_API_LIB}")
    install(FILES "${TSCAN_API_LIB}" "${TSCAN_HELPER_LIB}"
            DESTINATION bin)
endif()
