add_library(starsim_core STATIC
    src/time/utc_date_time.cpp
    src/frames/geodetic.cpp
    src/spice/spice_session.cpp
    src/spice/spice_time_converter.cpp
    src/spice/spice_ephemeris.cpp
    src/spice/spice_orientation.cpp
)
add_library(starsim::core ALIAS starsim_core)

target_include_directories(starsim_core PUBLIC include)
# CSPICE is an implementation detail of the spice adapter: never exposed in public headers.
target_link_libraries(starsim_core PRIVATE cspice)

if(MSVC)
    target_compile_options(starsim_core PRIVATE /W4 /permissive- /fp:precise)
else()
    target_compile_options(starsim_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
