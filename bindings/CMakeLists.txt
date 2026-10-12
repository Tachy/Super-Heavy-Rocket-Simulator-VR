# GDExtension: thin adapter between Godot (double-precision build) and starsim_core.

set(GODOT_CPP_DIR "${STARSIM_EXTERNAL_DIR}/godot-cpp")
set(GODOT_API_FILE "${GODOT_CPP_DIR}/extension_api.json")
if(NOT EXISTS "${GODOT_API_FILE}")
    message(FATAL_ERROR "Missing ${GODOT_API_FILE}. Dump it from our engine build:\n"
        "  cd external/godot-cpp && ../godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --dump-extension-api")
endif()

set(GODOTCPP_PRECISION "double" CACHE STRING "" FORCE)
set(GODOTCPP_CUSTOM_API_FILE "${GODOT_API_FILE}" CACHE FILEPATH "" FORCE)
set(GODOTCPP_TARGET "template_debug" CACHE STRING "" FORCE)
add_subdirectory("${GODOT_CPP_DIR}" "${CMAKE_BINARY_DIR}/godot-cpp" SYSTEM)

add_library(starsim_gdext SHARED
    src/register_types.cpp
    src/starsim_core.cpp
)
target_link_libraries(starsim_gdext PRIVATE starsim::core godot::cpp)

set(STARSIM_GAME_BIN_DIR "${CMAKE_SOURCE_DIR}/game/bin")
set_target_properties(starsim_gdext PROPERTIES
    OUTPUT_NAME "starsim.windows.template_debug.x86_64"
    RUNTIME_OUTPUT_DIRECTORY "${STARSIM_GAME_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${STARSIM_GAME_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${STARSIM_GAME_BIN_DIR}"
)
