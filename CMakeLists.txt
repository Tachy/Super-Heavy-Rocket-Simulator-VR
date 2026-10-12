cmake_minimum_required(VERSION 3.28)
project(starsim LANGUAGES C CXX)

option(STARSIM_BUILD_TESTS "Build core unit and validation tests" ON)
option(STARSIM_BUILD_BINDINGS "Build the Godot GDExtension (needs external/godot-cpp)" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# One static C/C++ runtime for everything (godot-cpp links it statically on Windows).
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>" CACHE STRING "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(STARSIM_EXTERNAL_DIR "${CMAKE_SOURCE_DIR}/external")
set(STARSIM_DATA_DIR "${CMAKE_SOURCE_DIR}/data")

include(cmake/cspice.cmake)

add_subdirectory(core)

if(STARSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(core/tests)
endif()

if(STARSIM_BUILD_BINDINGS)
    add_subdirectory(bindings)
endif()
